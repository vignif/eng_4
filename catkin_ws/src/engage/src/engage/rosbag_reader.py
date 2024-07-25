import seaborn as sns
import rospy
import numpy as np
import colorcet as cc
import cv2
import copy

from hri_msgs.msg import Skeleton2D

from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.robot_decision import RobotDecision
from engage.decision_maker.engage_state import EngageState
from engage.engage_state_grapher import EngageStateGrapher
from engage.marker_visualisation import MarkerMaker

class RosbagReader:
    def __init__(self,cam_bag,log_bag,pruning_threshold=10) -> None:
        self.cam_bag = cam_bag
        self.log_bag = log_bag

        # Get message types
        self.message_types,self.decision_type,self.state_type = self.get_message_types()

        # Read in the list of bodies
        self.bodies,self.body_count,self.body_times = self.read_bodies()

        # Prune bodies
        self.pruned_bodies = self.prune_bodies(pruning_threshold)

        # Colours
        self.colours = self.set_colours()

        # Locations
        self.locations,self.location_times,self.poses = self.get_locations(self.pruned_bodies)

         # Decisions
        self.decisions,self.decision_times,self.decision_msgs = self.read_decisions()

        # State
        self.states,self.state_times,self.state_bodies,self.decision_maker_name = self.read_decision_states()

        # Grapher
        self.grapher = self.get_grapher()


    def get_message_types(self):
        topic_dict = self.log_bag.get_type_and_topic_info()[1]
        message_types = {}

        for topic in topic_dict:
            message_types[topic] = topic_dict[topic][0]

        if "/hri_engage/decisions" not in message_types:
            error_message = "No decisions were taken in this rosbag"
            raise NotImplementedError(error_message)

        # Get decision type
        if message_types["/hri_engage/decisions"] == "engage/HeuristicDecision":
            decision_type = "heuristic"
        elif message_types["/hri_engage/decisions"] == "engage/RobotDecision":
            decision_type = "robot"
        else:
            error_message = "Unrecognised decision type: {}".format(message_types["/hri_engage/decisions"])
            raise NotImplementedError(error_message)
        
        # Get state type
        if message_types["/hri_engage/decision_states"] == "engage/HeuristicStateDecision":
            state_type = "heuristic"
        elif message_types["/hri_engage/decision_states"] == "engage/RobotStateDecision":
            state_type = "robot"
        else:
            error_message = "Unrecognised decision type: {}".format(message_types["/hri_engage/decisions"])
            raise NotImplementedError(error_message)

        return message_types,decision_type,state_type
    
    def get_grapher(self):
        if self.state_type == "heuristic" or self.state_type == "robot":
            return EngageStateGrapher()
    
    def read_bodies(self):
        # Create list of bodies in the rosbag
        body_count = {}
        body_times = {}
        for _,msg,_ in self.log_bag.read_messages(topics=["/humans/bodies/tracked"]):
            for body in msg.ids:
                if body not in body_count:
                    body_count[body] = 1
                    body_times[body] = [msg.header.stamp.to_sec()]
                else:
                    body_count[body] += 1
                    body_times[body].append(msg.header.stamp.to_sec())
        return list(body_count.keys()),body_count,body_times
    
    
    
    def prune_bodies(self,threshold):
        bodies = []
        for body in self.body_count:
            if self.body_count[body] >= threshold:
                bodies.append(body)
        return bodies
    
    def get_locations(self,bodies):
        locations = {body:[] for body in bodies}
        poses = {body:[] for body in bodies}
        times = {body:[] for body in bodies}
        for body in bodies:
            for _,msg,_ in self.log_bag.read_messages(topics=["/humans/bodies/{}/skeleton2d".format(body)]):
                times[body].append(msg.header.stamp.to_sec())
                poses[body].append(msg.skeleton)
                label_keypoint = (msg.skeleton[Skeleton2D.NOSE].x,msg.skeleton[Skeleton2D.NOSE].y)
                if label_keypoint == (-1,-1):
                    label_keypoint = (msg.skeleton[Skeleton2D.NECK].x,msg.skeleton[Skeleton2D.NECK].y)
                locations[body].append(label_keypoint)
        return locations,times,poses
    
    def read_decisions(self):
        topic = "/hri_engage/decisions"
        decision_msgs = []
        decisions = []
        times = []
        for _,msg,_ in self.log_bag.read_messages(topics=[topic]):
            if self.decision_type == "heuristic":
                decision_msgs.append(msg)

                decision = HeuristicDecision(msg.action,msg.target)
                dec_tuple = decision.decision_tuple_string()
                if msg.target == "":
                    decisions.append(dec_tuple[0])
                else:
                    decisions.append("{}_{}".format(dec_tuple[0],dec_tuple[1]))
                times.append(msg.header.stamp.to_sec())
            elif self.decision_type == "robot":
                decision_msgs.append(msg)

                decision = RobotDecision(msg.wait,msg.gesture,msg.gaze,msg.speech,msg.target)
                dec_tuple = decision.decision_tuple_string()
                decisions.append(decision.decision_string())
                times.append(msg.header.stamp.to_sec())
            else:
                error_message = "Can't process message of type: {}".format(self.message_types[topic])
                raise NotImplementedError(error_message)
        return decisions,times,decision_msgs
    
    def read_decision_states(self):
        topic = "/hri_engage/decision_states"
        if self.state_type == "robot":
            decision = RobotDecision
            decision_state = EngageState
        elif self.state_type == "heuristic":
            decision = HeuristicDecision
            decision_state = EngageState
        else:
            error_message = "Cannot read state type: {}".format(self.state_type)
            raise Exception(error_message)
        
        states,state_bodies,state_times = decision_state.create_state_dicts(decision)
        
        for _,msg,_ in self.log_bag.read_messages(topics=[topic]):
            states,state_bodies,state_times,decision_maker_name = decision_state.update_state_dicts_from_msg(states,state_bodies,state_times,msg,decision)

        return states,state_times,state_bodies,decision_maker_name
    
    '''
    PLOT
    '''

    def plot(self,graph_choice):
        return self.grapher.plot(graph_choice,self.bodies,self.pruned_bodies,self.log_bag,self.decisions,self.decision_times)
    
    '''
    LOCATIONS
    '''
    def get_face_locations(self,time):
        locations = {}
        for body in self.pruned_bodies:
            nearest_index = self.nearest_time_within_threshold(time,self.location_times[body])
            if nearest_index is not None:
                locations[body] = self.locations[body][nearest_index]
            else:
                locations[body] = None
        return locations
    
    '''
    DRAW
    '''
    def draw_poses(self,cv_img,curr_time):
        for body in self.poses:
            nearest_pose_index = self.nearest_time_within_threshold(curr_time.to_sec(),self.location_times[body])
            if nearest_pose_index is not None:
                nearest_pose = self.poses[body][nearest_pose_index]
                cv_img = self.draw_pose_on_image(cv_img,nearest_pose,body)
        return cv_img
    
    def draw_pose_on_image(self,cv_img,pose,body,thickness=2):
        col = 255*np.array(self.colours[body])
        for skeleton_pair in MarkerMaker.skeleton_pairs_indices:
            j0 = pose[skeleton_pair[0]]
            j1 = pose[skeleton_pair[1]]

            if (j0.x == -1 and j0.y == -1) or (j1.x == -1 and j1.y == -1):
                continue

            p0 = (int(cv_img.shape[1]*j0.x),int(cv_img.shape[0]*j0.y))
            p1 = (int(cv_img.shape[1]*j1.x),int(cv_img.shape[0]*j1.y))
            cv_img = cv2.line(cv_img, p0, p1, col, thickness)

        # Draw points for the ears even if the nose is not drawn
        points_to_draw = []
        if pose[Skeleton2D.NOSE].x == -1 and pose[Skeleton2D.NOSE].y == -1:
            if not (pose[Skeleton2D.LEFT_EAR].x == -1 and pose[Skeleton2D.LEFT_EAR].y == -1):
                point = (int(cv_img.shape[1]*pose[Skeleton2D.LEFT_EAR].x),int(cv_img.shape[0]*pose[Skeleton2D.LEFT_EAR].y))
                points_to_draw.append(point)
            if not (pose[Skeleton2D.RIGHT_EAR].x == -1 and pose[Skeleton2D.RIGHT_EAR].y == -1):
                point = (int(cv_img.shape[1]*pose[Skeleton2D.RIGHT_EAR].x),int(cv_img.shape[0]*pose[Skeleton2D.RIGHT_EAR].y))
                points_to_draw.append(point)

        for p in points_to_draw:
            cv_img = cv2.circle(cv_img, p, 1, col, thickness)

        return cv_img

    '''
    UTIL
    '''
    def nearest_time_within_threshold(self,time,timelist,threshold=0.1):
        if timelist == []:
            return None
        closest_index = np.argmin(np.abs(np.array(timelist)-time))
        if np.abs(timelist[closest_index] - time) > threshold:
            return None
        else:
            return closest_index
        
    def set_colours(self):
        colour = {}
        palette = sns.color_palette(cc.glasbey, n_colors=len(self.bodies))
        for i in range(len(self.bodies)):
            colour[self.bodies[i]] = palette[i]
        return colour