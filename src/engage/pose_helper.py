import rospy
import numpy as np
import math
from tsmoothie.smoother import LowessSmoother
import tf
import tf2_ros

from opendr.engine.target import Pose 

from engage.utils import RandomID,VectorHelper
from engage.marker_visualisation import MarkerMaker
from engage.msg import PoseArrayUncertain,PeoplePositions

from hri_msgs.msg import Skeleton2D, NormalizedPointOfInterest2D, IdsList
from sensor_msgs.msg import JointState
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker 
from geometry_msgs.msg import Vector3Stamped,TwistStamped,TransformStamped,Point
from geometry_msgs.msg import Pose as GPose

class HRIPoseBody:

    position_preference = [
        "nose",
        "l_ear",
        "r_ear",
        "l_eye",
        "r_eye",
        "neck",
        "l_sho",
        "r_sho",
        "l_hip",
        "r_hip",
        "l_elb",
        "r_elb",
        "l_wri",
        "r_wri",
        "l_knee",
        "r_knee",
        "l_ank",
        "r_ank",
    ]

    joints = {
        "nose":0,
        "neck":1,
        "r_sho":2,
        "r_elb":3,
        "r_wri":4,
        "l_sho":5,
        "l_elb":6,
        "l_wri":7,
        "r_hip":8,
        "r_knee":9,
        "r_ank":10,
        "l_hip":11,
        "l_knee":12,
        "l_ank":13,
        "r_eye":14,
        "l_eye":15,
        "r_ear":16,
        "l_ear":17,
    }

    def __init__(self,
                 id,
                 time,
                 visualise=False,
                 smoothing=False,
                 window=40,
                 marker_pub=None,
                 max_confidence=30,
                 camera_frame="camera",
                 world_frame="world"
                ):
        self.id = id
        self.start_time = time
        self.time = time
        self.visualise = visualise
        self.smoothing = smoothing
        self.window = window
        self.max_confidence = max_confidence
        self.camera_frame = camera_frame
        self.world_frame = world_frame

        if visualise:
            self.colour = list(np.random.choice(range(256), size=3)/256)
            self.marker_pub = marker_pub

        if smoothing:
            self.smoother = LowessSmoother(smooth_fraction=0.4, iterations=1)
            
        # Pose
        self.pose = None
        self.pose_confidence = 0
        self.pose_history = []
        for i in range(Pose.num_kpts):
            self.pose_history.append([])
        self.num_poses = 0
        self.position = Point()

        # Orientation
        self.body_normal = None
        self.face_normal = None
        self.num_normals = 0
        self.body_normal_history = []
        self.face_normal_history = []

        # Velocity
        self.velocity = None
        self.position_history = []
        self.position_times = []
        self.num_positions = 0

        # Transforms
        self.face_trans = [0,0,0]
        self.face_rot = [0,0,0,0]

        # Publishers
        this_body_topic = "/humans/bodies/{}/".format(self.id)
        self.pose_pub = rospy.Publisher(this_body_topic+"poses",PoseArrayUncertain,queue_size=1)
        self.vel_pub = rospy.Publisher(this_body_topic+"velocity",TwistStamped,queue_size=1)
        self.body_or_pub = rospy.Publisher(this_body_topic+"body_orientation",Vector3Stamped,queue_size=1)
        self.face_or_pub = rospy.Publisher(this_body_topic+"face_orientation",Vector3Stamped,queue_size=1)
        self.skeleton_pub = rospy.Publisher(this_body_topic+"skeleton2d",Skeleton2D,queue_size=1)

    def close(self):
        # To be called upon deletion
        self.pose_pub.unregister()
        self.vel_pub.unregister()
        self.body_or_pub.unregister()
        self.face_or_pub.unregister()
        self.skeleton_pub.unregister()

    def update(self,pose,time,rgb_image,rgb_model,depth_model,depth_image,cam_transform):
        self.pose = pose
        self.time = time

        # Update skeleton
        self.update_skeleton(rgb_image,depth_image)

        # Update 3D points
        self.update_3D_pose(rgb_model,depth_model,depth_image,cam_transform)

        # Update pose history
        self.update_pose_history()

        # Smoothing
        if self.smoothing:
            self.smooth_pose()

        # Calculate orientations
        self.update_orientations()

        # Calculate velocity
        self.update_velocity()

        # Set face transform
        self.update_face_transform()

        
    '''
    Poses
    '''

    def update_skeleton(self,rgb_image,depth_image):
        # Transform OpenDR pose into normalised 2D skeleton
        self.skeleton = Skeleton2D()
        self.skeleton.header.stamp = self.time

        self.skeleton.skeleton = [
            NormalizedPointOfInterest2D(
                kpt[0] / rgb_image.shape[1],
                kpt[1] / rgb_image.shape[0],
                self.pose.confidence
            ) 
            if (kpt[0] != -1 and kpt[1] != -1) else None for kpt in self.pose.data
        ]

    def update_3D_pose(self,rgb_model,depth_model,depth_image,inversed_transform):
        # Get coordinates in camera
        camera_coords = [self.point_2D_to_camera(kpt,rgb_model,depth_model,depth_image) for kpt in self.skeleton.skeleton]
        
        # Get world coords
        world_coords = []
        for cam_coord in camera_coords:
            if cam_coord is not None:
                world_coord = inversed_transform.dot(np.append(cam_coord,[1]))[0:3]
                
                if sum(world_coord)==0:
                    world_coord = None
            else:
                world_coord = None
            world_coords.append(world_coord)
        
        self.pose_3D = world_coords
        self.pose_confidence = min(1,self.pose.confidence / self.max_confidence) # OpenDR weirdly only goes up to about 30

    def update_pose_history(self):
        self.num_poses += 1
        overflow = False
        if self.num_poses > self.window:
            overflow = True

        for i in range(Pose.num_kpts):
            self.pose_history[i].append(self.pose_3D[i])
            if overflow:
                self.pose_history[i] = self.pose_history[i][-self.window:]
        if overflow:
            self.num_poses = self.window

    def update_face_transform(self):
        # Try get the nose point
        point = self.pose_3D[self.joints["nose"]]
        if point is None:
            point = self.pose_3D[self.joints["neck"]]
            if point is None:
                # Can't find either the nose or the neck, don't update the transform
                return None
            
        # Correct for ARI's weirdness
        trans = [point[0],point[1],point[2]]
        if self.camera_frame == "sellion_link" and self.world_frame == "base_link":
            trans[0] = abs(trans[0])
            trans[1] = -trans[1]
        
        
        # Update transformation
        self.face_trans = trans

    def get_position(self):
        for kp in self.position_preference:
            point = self.pose_3D[self.joints[kp]]
            if point is not None:
                self.position.x = point[0]
                self.position.y = point[1]
                self.position.z = point[2]
                # Correct for ARI's weirdness
                if self.camera_frame == "sellion_link" and self.world_frame == "base_link":
                    self.position.x = abs(self.position.x)
                    self.position.y = -self.position.y
                skeleton = self.skeleton.skeleton[self.joints[kp]]
                if skeleton is None:
                    skeleton = NormalizedPointOfInterest2D()
                    skeleton.x = -1
                    skeleton.y = -1
                    skeleton.c = 0
                return self.position,skeleton
        # No position
        self.position.x = 0
        self.position.y = 0
        self.position.z = 0
        skeleton = self.skeleton.skeleton[0]
        if skeleton is None:
            skeleton = NormalizedPointOfInterest2D()
            skeleton.x = -1
            skeleton.y = -1
            skeleton.c = 0
        return self.position,skeleton


    '''
    Orientations
    '''
    def update_orientations(self):
        self.body_normal = self.body_orientation()
        self.face_normal = self.face_orientation()
        self.update_normal_history()

        if self.smoothing:
            self.body_normal = self.smooth_vector(self.body_normal_history)
            self.face_normal = self.smooth_vector(self.face_normal_history)

    def body_orientation(self):
        # Get the two shoulders used for the normal
        points = []
        for joint_name in ["r_sho","l_sho"]:
            point = self.pose_3D[self.joints[joint_name]]
            if point is None:
                # The left and right shoulders are required for the body orientation
                return None
            points.append(point)

        # Find a third point
        l_hip = self.pose_3D[self.joints["l_hip"]]
        r_hip = self.pose_3D[self.joints["r_hip"]]
        if l_hip is not None and r_hip is not None:
            waist = (l_hip+r_hip)/2
            points.append(waist)
            return -VectorHelper.get_normal(points)
        elif self.pose_3D[self.joints["nose"]] is not None:
            points.append(self.pose_3D[self.joints["nose"]])
            return VectorHelper.get_normal(points)
        else:
            return None
        
    def face_orientation(self):
        # Get the two eyes and nose used for the normañ
        points = []
        for joint_name in ["r_eye","l_eye","nose"]:
            point = self.pose_3D[self.joints[joint_name]]
            if point is None:
                # The left and right eyes and the nose are required for the face orientation
                return None
            points.append(point.copy())

        # Replace nose depth with average of other depths
        points[-1][0] = (points[0][0]+points[1][0])/2
        return -VectorHelper.get_normal(points)
        
    def update_normal_history(self):
        self.num_normals += 1

        self.body_normal_history.append(self.body_normal)
        self.face_normal_history.append(self.face_normal)
        if self.num_normals > self.window:
            self.body_normal_history = self.body_normal_history[-self.window:]
            self.face_normal_history = self.face_normal_history[-self.window:]
            self.num_normals = self.window
        
    '''
    Velocity
    '''
    def update_velocity(self,base_point="neck"):
        self.velocity = self.calculate_velocity(base_point=base_point)


    def calculate_velocity(self,base_point="neck"):
        base = self.pose_3D[self.joints[base_point]]
        if base is None:
            return None
        

        if self.smoothing:
            # Use a history of smoothed positions
            self.num_positions += 1
            self.position_history.append(base)
            self.position_times.append(self.time.to_nsec())
            if self.num_positions > self.window:
                self.position_history = self.position_history[-self.window:]
                self.position_times = self.position_times[-self.window:]
                self.num_positions = self.window
            
            base_history = self.position_history
            base_history_num = self.num_positions
        else:
            # Use a history of unsmoothed positions
            base_history = self.pose_history[self.joints[base_point]]
            base_history_num = self.num_poses
            self.position_times.append(self.time.to_nsec())
            if len(self.position_times) > self.window:
                self.position_times = self.position_times[-self.window:]

        if base_history_num == self.window:
            num_vels = 0
            vel_sum = np.zeros(3)
            for i in range(1,base_history_num):
                if base_history[i] is None or base_history[i-1] is None:
                    continue
                pos_dif = base_history[i]-base_history[i-1]
                time_dif = self.position_times[i]-self.position_times[i-1]
                time_dif = time_dif/1000000000 # To seconds
                if time_dif != 0:
                    vel_sum += pos_dif/time_dif
                    num_vels += 1
            return vel_sum/num_vels if num_vels != 0 else None
        else:
            return None
                

            

        


    '''
    Projection
    '''

    def point_2D_to_camera(self,point,rgb_model,depth_model,depth_image):
        if point is None:
            return None
        
        px = min(math.floor(point.x * depth_image.shape[1]), depth_image.shape[1] - 1)
        py = min(math.floor(point.y * depth_image.shape[0]), depth_image.shape[0] - 1)

        x_d = int(((px - rgb_model.cx())
               * depth_model.fx()
               / rgb_model.fx())
              + depth_model.cx())
        y_d = int(((py - rgb_model.cy())
                * depth_model.fy()
                / rgb_model.fy())
                + depth_model.cy())
        z = depth_image[y_d][x_d]/1000

        x = (x_d - depth_model.cx())*z/depth_model.fx()
        y = (y_d - depth_model.cy())*z/depth_model.fy()


        return np.array([-z,x,-y])
    
    '''
    Visualise
    '''
    
    def visualise_pose(self):
        # Visualise skeleton
        self.visualise_skeleton()
        # Visualise normals
        self.visualise_normal(self.pose_3D[self.joints["neck"]],self.body_normal,18,[0,1,1])
        self.visualise_normal(self.pose_3D[self.joints["nose"]],self.face_normal,19,[1,0,1])
        # Visualise velocity
        self.visualise_velocity()
    
    def visualise_skeleton(self):
        for i in range(len(MarkerMaker.skeleton_pairs)):
            index_pair = MarkerMaker.skeleton_pairs_indices[i]
            line_marker = MarkerMaker.make_line_marker(
                self.pose_3D[index_pair[0]],
                self.pose_3D[index_pair[1]],
                self.id,
                i,
                colour=self.colour,
                frame_id=self.world_frame,
            )
            if line_marker is not None:
                self.marker_pub.publish(line_marker)

    def visualise_normal(self,start,normal,marker_id,colour=[0,0,0]):
        if start is None or normal is None:
            return
        
        mk = MarkerMaker.make_line_marker(start,start+normal,self.id,marker_id=marker_id,colour=colour,frame_id=self.world_frame)
        self.marker_pub.publish(mk)

    def visualise_velocity(self):
        self.visualise_normal(self.pose_3D[self.joints["neck"]],self.velocity,20,[0,1,0])

    '''
    Smoothing
    '''
    def smooth_pose(self):
        if self.num_poses==self.window:
            new_pose = []
            for i in range(self.pose.num_kpts):
                joint_history = [p for p in self.pose_history[i] if p is not None]
                if len(joint_history)==0:
                    new_pose.append(None)
                elif len(joint_history)==1:
                    new_pose.append(joint_history[0])
                else:
                    joint_history = np.stack(joint_history)
                    self.smoother.smooth(joint_history.T)
                    smoothed_pose = self.smoother.smooth_data[:,-1]
                    new_pose.append(smoothed_pose)
            self.pose_3D = new_pose

    def smooth_vector(self,vector_history):
        if len(vector_history)==self.window:
            vec_hist = [v for v in vector_history if v is not None]
            if len(vec_hist) == 0:
                return None
            elif len(vec_hist)==1:
                return vec_hist[0]
            else:
                vec_hist_arr = np.stack(vec_hist)
                self.smoother.smooth(vec_hist_arr.T)
                return self.smoother.smooth_data[:,-1]
        elif len(vector_history) == 0:
            return None
        else:
            return vector_history[-1]
        
    '''
    Publish
    '''
    def publish(self):
        # Publish skeleton
        for i in range(Pose.num_kpts):
            if self.skeleton.skeleton[i] is None:
                self.skeleton.skeleton[i] = NormalizedPointOfInterest2D(-1,-1,0)
        self.skeleton_pub.publish(self.skeleton)

        # Publish Pose
        pose = PoseArrayUncertain()
        pose.header.stamp = self.time
        pose.confidence = self.pose_confidence
        pose.header.frame_id = self.world_frame
        poses = []
        for joint_pose in self.pose_3D:
            if joint_pose is None:
                poses.append(GPose())
            else:
                this_pose = GPose()
                this_pose.position.x = joint_pose[0]
                this_pose.position.y = joint_pose[1]
                this_pose.position.z = joint_pose[2]
                poses.append(this_pose)
        pose.poses = poses
        self.pose_pub.publish(pose)

        # Publish Velocity
        velocity = TwistStamped()
        velocity.header.stamp = self.time
        if self.velocity is not None:
            velocity.twist.linear.x = self.velocity[0]
            velocity.twist.linear.y = self.velocity[1]
            velocity.twist.linear.z = self.velocity[2]
        self.vel_pub.publish(velocity)

        # Publish Body Orientation
        body_normal = Vector3Stamped()
        body_normal.header.stamp = self.time
        if self.body_normal is not None:
            body_normal.vector.x = self.body_normal[0]
            body_normal.vector.y = self.body_normal[1]
            body_normal.vector.z = self.body_normal[2]
        self.body_or_pub.publish(body_normal)

        # Publish Face Orientation
        face_normal = Vector3Stamped()
        face_normal.header.stamp = self.time
        if self.face_normal is not None:
            face_normal.vector.x = self.face_normal[0]
            face_normal.vector.y = self.face_normal[1]
            face_normal.vector.z = self.face_normal[2]
        self.face_or_pub.publish(face_normal)



    

class HRIPoseManager:
    def __init__(self,
                body_timeout=0.1,
                body_timein=0.1,
                visualise=True,
                smoothing=True,
                window=40,
                camera_frame="camera",
                world_frame="world"):
        # Parameters
        self.body_timeout = rospy.Duration(body_timeout)
        self.body_timein = rospy.Duration(body_timein)
        self.visualise = visualise
        self.smoothing = smoothing
        self.window = window
        self.camera_frame = camera_frame
        self.world_frame = world_frame
        # Map from pose id to body id
        self.body_ids = {}
        # Dict of all bodies currently tracked
        self.bodies = {}
        # Visualise
        if visualise:
            self.marker_pub = rospy.Publisher("/hri_engage/markers",Marker,queue_size=100)

        # Publishers
        self.body_pub = rospy.Publisher("/humans/bodies/tracked",IdsList,queue_size=1)
        self.position_pub = rospy.Publisher("/humans/bodies/positions",PeoplePositions,queue_size=1)

        # Initial Camera Projection
        trans = [0,0,0]
        rot = [0,0,0]
        self.update_camera_transform(trans,rot)

        # Transforms
        self.face_tf_br = tf.TransformBroadcaster()

    def update_camera_model(self,rgb_info,depth_info):
        self.depth_model = PinholeCameraModel()
        self.rgb_model = PinholeCameraModel()

        self.depth_model.fromCameraInfo(depth_info)
        self.rgb_model.fromCameraInfo(rgb_info)

    def update_camera_transform(self,trans,rot):
        transform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans), 
            tf.transformations.quaternion_matrix(rot)
        )
        self.inversed_transform = tf.transformations.inverse_matrix(transform)

    def broadcast_transforms(self):
        for body in self.bodies:
            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.world_frame
            t.child_frame_id = "{}_face_tf".format(body)
            t.transform.translation.x = self.bodies[body].face_trans[0]
            t.transform.translation.y = self.bodies[body].face_trans[1]
            t.transform.translation.z = self.bodies[body].face_trans[2]
            # TODO: Rotation, if this ever becomes important
            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            self.face_tf_br.sendTransformMessage(t)

    def publish_bodies(self,time):
        tracked = IdsList()
        bodies = []
        for id in self.bodies:
            bodies.append(id)
        tracked.ids = bodies
        tracked.header.stamp = time
        self.body_pub.publish(tracked)

    def publish_positions(self,time):
        pp = PeoplePositions()
        pp.header.stamp = time
        pp.bodies = [id for id in self.bodies]
        pose_skel = [self.bodies[body].get_position() for body in self.bodies]
        pp.positions = [p[0] for p in pose_skel]
        pp.points2d = [p[1] for p in pose_skel]
        print(pp)
        self.position_pub.publish(pp)

    def process_poses(self,poses,time,rgb_image,depth_image):
        bodies_in_pose = []
        curr_transform = self.inversed_transform.copy()

        # Start by updating the list of bodies
        for pose in poses:
            if pose.id not in self.body_ids:
                # New body
                self.body_ids[pose.id] = RandomID.random_id(pose.id)
                self.bodies[self.body_ids[pose.id]] = HRIPoseBody(
                    self.body_ids[pose.id],
                    time,
                    visualise=self.visualise,
                    smoothing=self.smoothing,
                    window=self.window,
                    marker_pub=self.marker_pub,
                    camera_frame=self.camera_frame,
                    world_frame=self.world_frame
                )
            self.bodies[self.body_ids[pose.id]].update(
                pose,
                time,
                rgb_image,
                self.rgb_model,
                self.depth_model,
                depth_image,
                curr_transform
            )
            bodies_in_pose.append(self.body_ids[pose.id])
        
        # Now, remove any bodies that have timed out
        for body in list(self.bodies.keys()):
            if body not in bodies_in_pose and abs(time - self.bodies[body].time) > self.body_timeout:
                self.bodies[body].close()
                del self.bodies[body]

        # Now publish the positions of each person
        self.publish_positions(time)

        # Now publish the updated body list
        self.publish_bodies(time)

        # Now publish the bodies
        for body in self.bodies:
            self.bodies[body].publish()

        # Now publish the transforms
        self.broadcast_transforms()

