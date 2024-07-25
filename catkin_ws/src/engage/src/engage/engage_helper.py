import numpy as np
import rospy
import tf
import networkx as nx
from collections import Counter

from engage.utils import VectorHelper,RandomID
from engage.pose_helper import HRIPoseBody

from message_filters import ApproximateTimeSynchronizer, Subscriber
from engage.msg import MotionActivity,Group,EngagementLevel,PoseArrayUncertain
from grace_common_msgs.msg import EngagementValue
from geometry_msgs.msg import TwistStamped,Vector3Stamped


class HRIEngageBody:
    def __init__(self,
                 id,
                 time,
                 window_size=20,
                 velocity_threshold=0.3,
                 face_cone=20,
                 velocity_angle=30,
                 camera_frame="camera",
                 world_frame="world"
                ):
        self.id = id
        self.time = time
        self.velocity_threshold=velocity_threshold
        self.face_cone=face_cone
        self.velocity_angle=velocity_angle
        self.window_size = window_size
        self.camera_frame = camera_frame
        self.world_frame = world_frame

        # Subscribers
        body_topic = "/humans/bodies/{}/".format(self.id)
        self.pose_sub = Subscriber(body_topic+"poses",PoseArrayUncertain)
        self.vel_sub = Subscriber(body_topic+"velocity",TwistStamped)
        self.body_or_sub = Subscriber(body_topic+"body_orientation",Vector3Stamped)
        self.face_or_sub = Subscriber(body_topic+"face_orientation",Vector3Stamped)

        time_slop = 1
        self.synch_sub = ApproximateTimeSynchronizer(
            [
                self.pose_sub,
                self.vel_sub,
                self.body_or_sub,
                self.face_or_sub
            ],
            10,
            time_slop,
        )
        self.synch_sub.registerCallback(self.update_body)

        # Publishers
        self.engagement_publisher = rospy.Publisher(body_topic+"engagement_status",EngagementLevel,queue_size=1)
        self.activity_publisher = rospy.Publisher(body_topic+"activity",MotionActivity,queue_size=1)

        # Low level variables
        self.pose = None
        self.pose_confidence = 0
        self.position = None
        self.velocity = None
        self.body_norm = None
        self.face_norm = None

        # High level variables
        self.distance_to_robot = None
        self.mutual_gaze_robot = None
        self.engagement_robot = None
        self.engagement_history = []
        self.engagement_history_size = window_size
        self.engagement_level = EngagementLevel.UNKNOWN
        self.engagement_confidence = 0
        self.update_robot_engagement(0)

        # Motion
        self.motion = MotionActivity.NOTHING
        self.motion_confidence = 0
        self.motion_history = []
        self.motion_history_size = window_size

    def __str__(self) -> str:
        return self.id

    def close(self):
        # Unregister subscribers
        self.pose_sub.sub.unregister()
        self.vel_sub.sub.unregister()
        self.body_or_sub.sub.unregister()
        self.face_or_sub.sub.unregister()
        # Unregister publishers
        self.engagement_publisher.unregister()
        self.activity_publisher.unregister()

    def update_body(self,pose_msg,vel_msg,body_or_msg,face_or_msg):
        # Update pose
        # TODO: How to handle positions that are not detected, as in the world frame 0,0,0 can be valid. Very large numbers?
        self.pose = []
        self.pose_confidence = pose_msg.confidence
        for pose in pose_msg.poses:
            pt = pose.position
            if sum([pt.x,pt.y,pt.z]) == 0:
                self.pose.append(None)
            else:
                self.pose.append(VectorHelper.obj_to_vec(pt))
        if self.pose[HRIPoseBody.joints["neck"]] is not None:
            self.position = self.pose[HRIPoseBody.joints["neck"]]
        elif self.pose[HRIPoseBody.joints["nose"]] is not None:
            self.position = self.pose[HRIPoseBody.joints["nose"]]
        else:
            self.position = None

        # Update velocity
        self.velocity = VectorHelper.obj_to_vec(vel_msg.twist.linear)

        # Update body norm
        bnorm = body_or_msg.vector
        if sum([bnorm.x,bnorm.y,bnorm.z]) == 0:
            self.body_norm = None
        else:
            self.body_norm = VectorHelper.obj_to_vec(bnorm)

        # Update face norm
        fnorm = face_or_msg.vector
        if sum([fnorm.x,fnorm.y,fnorm.z]) == 0:
            self.face_norm = None
        else:
            self.face_norm = VectorHelper.obj_to_vec(fnorm)

    def update_time(self,time):
        self.time = time

    def update_robot_engagement(self,engagement):
        if len(self.engagement_history) < self.engagement_history_size:
            # Not enough data
            if engagement != 0:
                self.engagement_history.append(engagement)
        else:
            # Add new data point
            self.engagement_history.append(engagement)
            # Keep window size the same
            self.engagement_history = self.engagement_history[-self.engagement_history_size:]
            # Average engagement
            average_engagement = sum(self.engagement_history)/self.engagement_history_size

            # Update engagement level
            old_engagement = self.engagement_level
            if old_engagement == EngagementLevel.UNKNOWN:
                # Current engagement unknown, set to disengaged
                self.engagement_level = EngagementLevel.DISENGAGED
            elif old_engagement == EngagementLevel.DISENGAGED:
                # Currently disengaged
                if 0 <= average_engagement <= 1:
                    # Upward trend, start engaging
                    self.engagement_level = EngagementLevel.ENGAGING
            elif old_engagement == EngagementLevel.ENGAGING:
                # Currently starting to engage
                if -1 <= average_engagement < -0.5:
                    # Downward trend, disengage
                    self.engagement_level = EngagementLevel.DISENGAGED
                elif 0.5 < average_engagement <= 1.0:
                    # Continued upward trend, engaged
                    self.engagement_level = EngagementLevel.ENGAGED
            elif old_engagement == EngagementLevel.ENGAGED:
                # Currently engaged
                if average_engagement <= 0.5:
                    # Downward trend, start disengaging
                    self.engagement_level = EngagementLevel.DISENGAGING
            elif old_engagement == EngagementLevel.DISENGAGING:
                # Currently disengaging
                if average_engagement > 0.5:
                    # Upward trend, engaged
                    self.engagement_level = EngagementLevel.ENGAGED
                elif average_engagement <= 0.0:
                    # Continued downward trend, disengage
                    self.engagement_level = EngagementLevel.DISENGAGED
        variance = np.var(self.engagement_history)
        self.engagement_confidence = 1-variance

    '''
    Motion
    '''

    def update_motion(self,robot_position):
        # Check if moving
        if self.velocity is not None and np.linalg.norm(self.velocity) > self.velocity_threshold:
            if self.walking_towards_robot(robot_position):
                curr_activity = MotionActivity.WALKING_TOWARDS
            elif self.walking_away_from_robot(robot_position):
                curr_activity = MotionActivity.WALKING_AWAY
            else:
                curr_activity = MotionActivity.WALKING_PAST
        else:
            curr_activity = MotionActivity.NOTHING

        self.motion_history.append(curr_activity)
        if len(self.motion_history) > self.motion_history_size:
            self.motion_history = self.motion_history[-self.motion_history_size:]

        if len(self.motion_history) == self.motion_history_size:
            counted = Counter(self.motion_history).most_common(1)
            self.motion = counted[-1][0]
            self.motion_confidence = counted[-1][1]/self.motion_history_size

    def walking_towards_robot(self,robot_position):
        return VectorHelper.facing_point(self.velocity,self.position,robot_position,self.velocity_angle)

    def walking_away_from_robot(self,robot_position):
        return VectorHelper.facing_point(-self.velocity,self.position,robot_position,self.velocity_angle)
            

    '''
    Publishing
    '''
    def publish_engagement(self):
        eng_msg = EngagementLevel()
        eng_msg.header.stamp = self.time
        eng_msg.level = self.engagement_level
        eng_msg.confidence = self.engagement_confidence
        self.engagement_publisher.publish(eng_msg)

    def publish_motion(self):
        activity_msg = MotionActivity()
        activity_msg.activity = self.motion
        activity_msg.confidence = self.motion_confidence
        activity_msg.header.stamp = self.time
        self.activity_publisher.publish(activity_msg)



class HRIEngagementManager:
    def __init__(self,
                 engagement_threshold=0.55,
                 max_mutual_gaze_angle=np.pi,
                 window_size=20,
                 ignore_z=True,
                 velocity_threshold=0.3,
                 face_cone=20,
                 velocity_angle=30,
                 camera_frame="camera",
                 world_frame="world"
                 ):
        # Parameters
        self.engagement_threshold = engagement_threshold
        self.max_mutual_gaze_angle = max_mutual_gaze_angle
        self.window_size = window_size
        self.velocity_threshold=velocity_threshold
        self.face_cone=face_cone
        self.velocity_angle=velocity_angle
        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.ignore_z = ignore_z

        # Time
        self.time = None

        # Managed dicts
        self.bodies = {}
        self.groups = {}

        # Robot's position and orientation
        self.robot_position = [0,0,0]
        self.robot_orientation = [-1,0,0]

        # Publishers
        self.engagement_value_publisher = rospy.Publisher("/humans/interactions/engagements",EngagementValue,queue_size=100)
        self.group_publisher = rospy.Publisher("/humans/interactions/groups",Group,queue_size=100)

        # Debug
        self.num_bodies_added = 0

    def update_robot_position(self,trans,rot):
        robot_position_camera = np.array([0,0,0,1])
        robot_orientation_camera = np.array([-1,0,0,1])

        # Get transform
        transform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans), 
            tf.transformations.quaternion_matrix(rot)
        )
        inversed_transform = tf.transformations.inverse_matrix(transform)

        # Robot position in world coords
        self.robot_position = inversed_transform.dot(robot_position_camera)[0:3]

        # Robot orientation vector
        robot_gaze_point = inversed_transform.dot(robot_orientation_camera)[0:3]
        self.robot_orientation = robot_gaze_point - self.robot_position
        
        

    def manage_bodies(self,time,ids):
        self.time = time

        tracked_ids = set(ids)
        managed_ids = set(self.bodies.keys())

        # Add new ids
        ids_to_add = list(tracked_ids - managed_ids)
        for id in ids_to_add:
            self.num_bodies_added += 1
            self.bodies[id] = HRIEngageBody(
                id,
                time,
                window_size=self.window_size,
                velocity_threshold=self.velocity_threshold,
                velocity_angle=self.velocity_angle,
                face_cone=self.face_cone,
                camera_frame=self.camera_frame,
                world_frame=self.world_frame
                )
            self.groups[id] = None
        if "ROBOT" not in self.groups:
            self.groups["ROBOT"] = None
        
        # Remove old bodies
        ids_to_rem = list(managed_ids - tracked_ids)
        for id in ids_to_rem:
            self.bodies[id].close()
            del self.bodies[id]
            del self.groups[id]

        # Update time
        for body in self.bodies:
            self.bodies[body].update_time(time)

    '''
    Engagement
    '''

    def calculate_engagement(self):
        body_keys = list(self.bodies.keys())
        num_bodies = len(body_keys)
        body_indices = {body_keys[i]:i for i in range(num_bodies)}
        body_list = body_keys + ["ROBOT"]
        pairs = [(a, b) for idx, a in enumerate(body_list) for b in body_list[idx + 1:]]

        engagements = np.zeros((num_bodies,num_bodies))
        distances = np.zeros((num_bodies,num_bodies))
        mutual_gazes = np.zeros((num_bodies,num_bodies))

        robot_position = self.robot_position
        robot_orientation = self.robot_orientation
        
        for pair in pairs:
            body_a = self.bodies[pair[0]]
            '''
            Note: as it stands, the human-robot mutual gaze uses faces and human-human uses body. In general, faces are harder to detect, but more reliable when they are available.
            TODO: Maybe try use face and switch to body if faces can't be used?
            '''
            if pair[1] == "ROBOT":
                distance = VectorHelper.distance(body_a.position,robot_position)
                _,_,mutual_gaze = self.mutual_gaze(body_a.position,robot_position,body_a.face_norm,robot_orientation)
                engagement = self.engagement(distance,mutual_gaze)

                body_a.distance_to_robot = distance
                body_a.mutual_gaze_robot = mutual_gaze
                body_a.engagement_robot = engagement

                if engagement > self.engagement_threshold:
                    body_a.update_robot_engagement(1)
                else:
                    body_a.update_robot_engagement(-1)
            else:
                body_b = self.bodies[pair[1]]
                distance = VectorHelper.distance(body_a.position,body_b.position)
                _,_,mutual_gaze = self.mutual_gaze(body_a.position,body_b.position,body_a.body_norm,body_b.body_norm)
                engagement = self.engagement(distance,mutual_gaze)

                distances[body_indices[body_a.id],body_indices[body_b.id]] = distance
                mutual_gazes[body_indices[body_a.id],body_indices[body_b.id]] = mutual_gaze
                engagements[body_indices[body_a.id],body_indices[body_b.id]] = engagement

        return distances,mutual_gazes,engagements


    def mutual_gaze(self,pos_a,pos_b,or_a,or_b):
        vecs = [pos_a,pos_b,or_a,or_b]
        if self.ignore_z:
            for i in range(len(vecs)):
                if vecs[i] is not None:
                    vecs[i][2] = 0

        if vecs[0] is None or vecs[1] is None:
            return None,None,None

        # A's gaze
        if vecs[2] is not None:
            pos_ba = vecs[1]-vecs[0]
            angle_a2b = VectorHelper.angle_between(vecs[2],pos_ba)
            gaze_a2b = max(0,1-(angle_a2b/self.max_mutual_gaze_angle))
        else:
            gaze_a2b = None

        # B's gaze
        if vecs[3] is not None:   
            pos_ab = vecs[0]-vecs[1]
            angle_b2a = VectorHelper.angle_between(vecs[3],pos_ab)
            gaze_b2a = max(0,1-(angle_b2a/self.max_mutual_gaze_angle))
        else:
            gaze_b2a = None

        # Mutual Gaze
        if gaze_a2b is not None and gaze_b2a is not None:
            mutual_gaze = gaze_a2b*gaze_b2a
        else:
            mutual_gaze = None

        return gaze_a2b,gaze_b2a,mutual_gaze
    
    def engagement(self,distance,mutual_gaze):
        if distance is None or distance == 0 or mutual_gaze is None:
            return 0
        return min(1,mutual_gaze/distance)
    
    '''
    Motion
    '''
    def calculate_motion(self):
        for body in self.bodies:
            self.bodies[body].update_motion(self.robot_position)
    
    '''
    Groups
    '''
    def calculate_groups(self,engagements):
        self.create_group_graph(engagements)
        group_sizes = self.divide_into_groups()
        group_confidences = self.group_confidences(group_sizes,engagements)
        return group_confidences

    def create_group_graph(self,engagements):
        self.group_graph = nx.Graph()
        body_keys = list(self.bodies.keys())
        self.group_graph.add_nodes_from(body_keys)
        self.group_graph.add_node("ROBOT")
        for i in range(len(body_keys)):
            # Check for other bodies
            for j in range(i+1,len(body_keys)):
                if engagements[i,j] > self.engagement_threshold:
                    self.group_graph.add_edge(body_keys[i],body_keys[j],weight=engagements[i,j])
            # Check for robot
            if self.bodies[body_keys[i]].engagement_level == EngagementLevel.ENGAGED:
                self.group_graph.add_edge(body_keys[i],"ROBOT",weight=self.bodies[body_keys[i]].engagement_confidence)

    def divide_into_groups(self):
        sub_graphs = [self.group_graph.subgraph(c) for c in nx.connected_components(self.group_graph)]
        used_g_ids = []
        group_sizes = {}
        curr_group_id = 100000
        for sg in sub_graphs:
            sg_list = list(sg)
            existing_groups = [self.groups[id] for id in sg.nodes]
            pruned_list = [g for g in existing_groups if g is not None]
            if pruned_list == []:
                # Nobody in the group already had a group, make a new one
                g_id = RandomID.random_id(curr_group_id)
                curr_group_id += 1
            else:
                if pruned_list[0] in used_g_ids:
                    # Old group name already in use (e.g. 1 group split into 2), must create new one
                    g_id = RandomID.random_id(curr_group_id)
                    curr_group_id += 1
                else:
                    g_id = pruned_list[0]
            used_g_ids.append(g_id)
            for id in sg.nodes:
                self.groups[id] = g_id

            group_sizes[g_id] = len(sg_list)
        return group_sizes
    
    def group_confidences(self,group_sizes,engagements):
        # Group confidences
        group_confidences = {}
        body_keys = list(self.bodies.keys())
        robot_engagements = [0]
        for i in range(len(body_keys)):
            body = self.bodies[body_keys[i]]
            max_engagement = max(max(engagements[i,:]),max(engagements[:,i]),body.engagement_robot)
            robot_engagements.append(body.engagement_robot)
            if group_sizes[self.groups[body.id]] == 1:
                # Only the one individual
                group_confidences[body.id] = 1 - max_engagement
            else:
                # Part of a group
                group_confidences[body.id] = max_engagement
        
        # Robot group confidence
        max_robot_engagement = max(robot_engagements)
        if group_sizes[self.groups["ROBOT"]] == 1:
            group_confidences["ROBOT"] = 1 - max_robot_engagement
        else:
            group_confidences["ROBOT"] = max_robot_engagement

        return group_confidences

    '''
    Publishing
    '''

    def publish_engagements(self,distances,mutual_gazes,engagements):
        body_keys = list(self.bodies.keys())
        for i in range(len(body_keys)):
            body1 = self.bodies[body_keys[i]]

            # Publish engagement level
            body1.publish_engagement()

            if body1.position is None:
                continue

            # Publish engagement with robot
            robot_eng = EngagementValue()
            robot_eng.person_a = body1.id
            if body1.distance_to_robot is not None:
                robot_eng.distance = body1.distance_to_robot
            if body1.mutual_gaze_robot is not None: 
                robot_eng.mutual_gaze = body1.mutual_gaze_robot
            if body1.engagement_robot is not None: 
                robot_eng.engagement = body1.engagement_robot
            robot_eng.header.stamp = self.time
            robot_eng.confidence_a = body1.pose_confidence
            robot_eng.confidence_b = 1
            self.engagement_value_publisher.publish(robot_eng)


            for j in range(i+1,len(body_keys)):
                body2 = self.bodies[body_keys[j]]

                if body2.position is None:
                    continue

                eng_ij = EngagementValue()
                eng_ij.person_a = body1.id
                eng_ij.person_b = body2.id
                if distances[i,j] is not None:
                    eng_ij.distance = distances[i,j]
                if mutual_gazes[i,j] is not None:
                    eng_ij.mutual_gaze = mutual_gazes[i,j]
                if engagements[i,j] is not None:
                    eng_ij.engagement = engagements[i,j]
                eng_ij.header.stamp = self.time
                eng_ij.confidence_a = body1.pose_confidence
                eng_ij.confidence_b = body2.pose_confidence
                self.engagement_value_publisher.publish(eng_ij)

            
    def publish_groups(self,group_confidences):
        groups = list(set(self.groups.values()))
        for group in groups:
            if group is None:
                continue
            members = [id for id,g in self.groups.items() if g==group]
            confidences = [group_confidences[id] for id in members]
            group_msg = Group()
            group_msg.group_id = group
            group_msg.members = members
            group_msg.confidences = confidences
            group_msg.header.stamp = self.time
            self.group_publisher.publish(group_msg)

    def publish_motion(self):
        for body in self.bodies:
            self.bodies[body].publish_motion()