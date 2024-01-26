import matplotlib.pyplot as plt
from itertools import combinations
import seaborn as sns
import rospy
import numpy as np
import colorcet as cc
import cv2
import copy

from engage.message_helper import MessageHelper
from engage.marker_visualisation import MarkerMaker
from engage.decision_helper import DecisionState
from hri_msgs.msg import Skeleton2D

CMAP = plt.get_cmap("tab10")

class Bagreader:   
    single_value_graphs = [
        "Engagement Level with Robot",
        "Engagement Level Confidence",
        "Motion Activity",
        "Motion Activity Confidence",
    ]

    engage_value_graphs = [
        "Distance to Robot",
        "Engagement Score with Robot",
        "Mutual Gaze with Robot",
        "Pose Estimation Confidence",
    ]

    pairwise_graphs = [
        "Pairwise Distance",
        "Pairwise Engagement Score",
        "Pairwise Mutual Gaze",
    ]

    special_graphs = [
        "Decision",
        "Group Confidence",
        "Group Membership",
    ]

    graph_choices = sorted(single_value_graphs + engage_value_graphs + pairwise_graphs + special_graphs)


    def __init__(self,cam_bag,log_bag,pruning_threshold=10):
        self.cam_bag = cam_bag
        self.log_bag = log_bag

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
        self.states,self.state_times,self.state_bodies = self.read_decision_states()

        

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
            decision_msgs.append(msg)
            if msg.target == "":
                decisions.append(MessageHelper.decision_names[msg.decision])
            else:
                decisions.append("{}_{}".format(MessageHelper.decision_names[msg.decision],msg.target))
            times.append(msg.header.stamp.to_sec())
        return decisions,times,decision_msgs

    
    def graph_choice(self):
        return self.graph_choices
    
    def read_decision_states(self):
        # Set up state dict
        states = {}
        state_bodies = []
        state_times = {}
        states["ROBOT"] = {"Group":[],"Group Confidence":[]}
        states["GENERAL"] = {"Waiting":[]}
        states["DECISION"] = {"Action":[],"Target":[]}
        state_times["ROBOT"] = []

        for _,msg,_ in self.log_bag.read_messages(topics=["/hri_engage/decision_states"]):

            for i in range(len(msg.bodies)):
                body = msg.bodies[i]

                if body not in states:
                    states[body] = {}
                    state_times[body] = []
                    states[body]["Distance"] = []
                    states[body]["Mutual Gaze"] = []
                    states[body]["Engagement Value"] = []
                    states[body]["Pose Estimation Confidence"] = []
                    states[body]["Engagement Level"] = []
                    states[body]["Engagement Level Confidence"] = []
                    states[body]["Motion"] = []
                    states[body]["Motion Confidence"] = []
                    states[body]["Group"] = []
                    states[body]["Group Confidence"] = []
                    states[body]["Group with Robot"] = []

                state_times[body].append(msg.header.stamp.to_sec())
                states[body]["Distance"].append(msg.distances[i])
                states[body]["Mutual Gaze"].append(msg.mutual_gazes[i])
                states[body]["Engagement Value"].append(msg.engagement_values[i])
                states[body]["Pose Estimation Confidence"].append(msg.pose_estimation_confidences[i])
                states[body]["Engagement Level"].append(msg.engagement_levels[i])
                states[body]["Engagement Level Confidence"].append(msg.engagement_level_confidences[i])
                states[body]["Motion"].append(msg.motion_activities[i])
                states[body]["Motion Confidence"].append(msg.motion_activity_confidences[i])
                states[body]["Group"].append(msg.groups[i])
                states[body]["Group Confidence"].append(msg.group_confidences[i])
                states[body]["Group with Robot"].append(msg.group_with_robot[i])

            states["GENERAL"]["Waiting"].append(msg.waiting)
            states["DECISION"]["Action"].append(msg.decision_action)
            states["DECISION"]["Target"].append(msg.decision_target)
            states["ROBOT"]["Group"].append(msg.robot_group)
            states["ROBOT"]["Group Confidence"].append(msg.robot_group_confidence)
            state_times["ROBOT"].append(msg.header.stamp.to_sec())
            state_bodies.append(msg.bodies)

        return states,state_times,state_bodies

    
    '''
    PLOT
    '''

    def plot(self,graph_name):
        if graph_name in self.special_graphs:
            # Graphs that have special features
            if graph_name == "Decision":
                fig,axes = self.plot_decision(graph_name)
            elif graph_name == "Group Confidence":
                fig,axes = self.plot_group_confidence(graph_name,self.pruned_bodies)
            elif graph_name == "Group Membership":
                fig,axes = self.plot_group_membership(graph_name,self.pruned_bodies)
        elif graph_name in self.engage_value_graphs:
            # Graphs that contain one line for each body and use the EngagementValue message
            fig,axes = self.plot_engagement_value_msg(graph_name,self.pruned_bodies)
        elif graph_name in self.pairwise_graphs:
            # Graphs for pairwise plots between bodies
            fig,axes = self.plot_pairwise(graph_name,self.pruned_bodies)
        elif graph_name in self.single_value_graphs:
            # Graphs for single values per body
            fig,axes = self.plot_single_values(graph_name,self.pruned_bodies)

        return fig,axes
    
    def plot_single_values(self,graph_name,bodies):   
        if graph_name == "Engagement Level with Robot":
            subtopic = "engagement_status"
            label = "Engagement Level"
        elif graph_name == "Engagement Level Confidence":
            subtopic = "engagement_status"
            label = "Confidence"
        elif graph_name ==  "Motion Activity":
            subtopic = "activity"
            label = "Motion Activity"
        elif graph_name == "Motion Activity Confidence":
            subtopic = "activity"
            label = "Confidence"
        
        values = {body:[] for body in bodies}
        times = {body:[] for body in bodies}

        for body in bodies:
            for _,msg,_ in self.log_bag.read_messages(topics=["/humans/bodies/{}/{}".format(body,subtopic)]):
                if graph_name == "Engagement Level with Robot":
                    values[body].append(msg.level)
                elif graph_name == "Engagement Level Confidence":
                    values[body].append(msg.confidence)
                elif graph_name ==  "Motion Activity":
                    values[body].append(msg.activity)
                elif graph_name == "Motion Activity Confidence":
                    values[body].append(msg.confidence)
                times[body].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel(label)


        if graph_name == "Engagement Level with Robot":
            unique_decisions = MessageHelper.engagement_level_names
            plt.yticks(list(range(len(unique_decisions))),unique_decisions,rotation=45)
        elif graph_name == "Motion Activity":
            unique_decisions = MessageHelper.motion_activity_names
            plt.yticks(list(range(len(unique_decisions))),unique_decisions,rotation=45)

        for body in bodies:
            plt.plot(times[body],values[body],label=body)
        plt.legend()

        return fig,[ax]

    
    def plot_pairwise(self,graph_name,bodies):
        topic = "/humans/interactions/engagements"
        body_combos = list(combinations(bodies+["ROBOT"], 2))
        values = {"{}_{}".format(bc[0],bc[1]):[] for bc in body_combos}
        times = {"{}_{}".format(bc[0],bc[1]):[] for bc in body_combos}

        for _,msg,_ in self.log_bag.read_messages(topics=[topic]):
            body_a = msg.person_a
            body_b = msg.person_b if msg.person_b != "" else "ROBOT"
            combo = "{}_{}".format(body_a,body_b)
            if combo not in values:
                combo = "{}_{}".format(body_b,body_a)
                if combo not in values:
                    continue


            if graph_name == "Pairwise Distance":
                values[combo].append(msg.distance)
                label = "Distance"
            elif graph_name == "Pairwise Engagement Score":
                values[combo].append(msg.engagement)
                label = "Engagement Score"
            elif graph_name == "Pairwise Mutual Gaze":
                values[combo].append(msg.mutual_gaze)
                label = "Mutual Gaze"
            else:
                raise NotImplementedError

            times[combo].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel(label)

        for combo_pair in body_combos:
            combo = "{}_{}".format(combo_pair[0],combo_pair[1])
            plt.plot(times[combo],values[combo],label=combo)
        ax.legend()

        return fig,[ax]
                
    def plot_engagement_value_msg(self,graph_name,bodies):
        topic = "/humans/interactions/engagements"
        values = {body:[] for body in bodies}
        times = {body:[] for body in bodies}
        for _,msg,_ in self.log_bag.read_messages(topics=[topic]):
            if msg.person_b == "" and msg.person_a in values: #ASSUMPTION: if robot is present, is always the second person
                # Involves the robot
                body = msg.person_a
                if graph_name == "Distance to Robot":
                    values[body].append(msg.distance)
                    label = "Distance"
                elif graph_name == "Engagement Score with Robot":
                    values[body].append(msg.engagement)
                    label = "Engagement Score"
                elif graph_name == "Mutual Gaze with Robot":
                    values[body].append(msg.mutual_gaze)
                    label = "Mutual Gaze"
                elif graph_name == "Pose Estimation Confidence":
                    values[body].append(msg.confidence_a)
                    label = "Confidence"
                else:
                    raise NotImplementedError

                times[body].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel(label)

        for body in bodies:
            plt.plot(times[body],values[body],label=body)
        ax.legend()

        return fig,[ax]
    
    def plot_group_confidence(self,graph_name,bodies):
        topic = "/humans/interactions/groups"
        values = {body:[] for body in bodies+["ROBOT"]}
        times = {body:[] for body in bodies+["ROBOT"]}
        for _,msg,_ in self.log_bag.read_messages(topics=[topic]):
            for member,confidence in zip(msg.members,msg.confidences):
                values[member].append(confidence)
                times[member].append(msg.header.stamp.to_sec())
        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("Confidence")

        for body in bodies+["ROBOT"]:
            plt.plot(times[body],values[body],label=body)
        ax.legend()

        return fig,[ax]
    
    def plot_group_membership(self,graph_name,bodies):
        topic = "/humans/interactions/groups"
        values = {}
        times = {}
        
        these_bodies = bodies+["ROBOT"]
        body_indices = {body:these_bodies.index(body) for body in these_bodies}

        for _,msg,_ in self.log_bag.read_messages(topics=[topic]):
            if len(msg.members)==0 or len(msg.members)==1:
                continue
            if msg.group_id not in values:
                values[msg.group_id] = {}
                times[msg.group_id] = {}
            for body in msg.members:
                if body in body_indices:
                    if body not in values[msg.group_id]:
                        values[msg.group_id][body] = []
                        times[msg.group_id][body] = []
                    values[msg.group_id][body].append(body_indices[body])
                    times[msg.group_id][body].append(msg.header.stamp.to_sec())

            for body in values[msg.group_id]:
                if body not in msg.members:
                    # Has left the group
                    values[msg.group_id][body].append(None)
                    times[msg.group_id][body].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("Body")

        group_list = list(values.keys())
        plotted_groups = []

        plt.yticks(list(range(len(these_bodies))),these_bodies,rotation="horizontal")
        palette = sns.color_palette(None, len(group_list))
        for group in values:
            colour = palette[group_list.index(group)]
            for body in values[group]:
                plotted_groups.append(group)
                plt.plot(times[group][body],values[group][body],color=colour,label=group, linewidth=1)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return fig,[ax]
                    
        

    def plot_decision(self,graph_name):
        unique_decisions = sorted(list(set(self.decisions)))
        index_dict = {decision:unique_decisions.index(decision) for decision in unique_decisions}
        decision_enums = [index_dict[decision] for decision in self.decisions]
        fig,ax = plt.subplots()

        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("Decision")
        plt.yticks(list(range(len(unique_decisions))),unique_decisions,rotation=45)

        plt.plot(self.decision_times,decision_enums)

        return fig,[ax]
    
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
    DECISIONS
    '''
    def get_decision(self,time):
        nearest_index = self.nearest_time_within_threshold(time,self.decision_times)
        msg = self.decision_msgs[nearest_index]
        action = MessageHelper.decision_names[msg.decision]
        target = msg.target
        target = None if target == "" else target
        return action,target
    
    def get_state(self,time,time_threshold=0.05):
        state = {}
        

        state_index = self.nearest_time_within_threshold(time,self.state_times["ROBOT"],time_threshold)
        if state_index is not None:
            for key in self.states:
                state[key] = {}
                for var in self.states[key]:
                    if key == "ROBOT" or key == "GENERAL" or key == "DECISION":
                        state[key][var] = self.states[key][var][state_index]
                    else:
                        var_list = self.states[key][var]
                        if self.state_times["ROBOT"][state_index] in self.state_times[key]:
                            state[key][var] = var_list[self.state_times[key].index(self.state_times["ROBOT"][state_index])]

        keys_to_del = []
        for key in state:
            if state[key] == {}:
                keys_to_del.append(key)
        for key in keys_to_del:
            del state[key]

        state_discrete = self.discretise_state(state,self.state_bodies[state_index])
        state_readable = self.make_state_readable(state,state_discrete)


        return state,state_readable,state_discrete,self.state_bodies[state_index]


    def discretise_state(self,state,bodies):
        discrete_state = copy.deepcopy(state)

        # Remove decision
        del discrete_state["DECISION"]

        discrete_state["ROBOT"]["Group Confidence"] = DecisionState.float_bucket(discrete_state["ROBOT"]["Group Confidence"])

        body_floats = ["Mutual Gaze","Engagement Value","Pose Estimation Confidence","Engagement Level Confidence","Motion Confidence","Group Confidence"]

        for body in bodies:
            discrete_state[body]["Distance"] = DecisionState.distance_bucket(discrete_state[body]["Distance"])
            for bf in body_floats:
                discrete_state[body][bf] = DecisionState.float_bucket(discrete_state[body][bf])
        return discrete_state
    
    def make_state_readable(self,state,discrete_state):
        readable_state = copy.deepcopy(state)

        # Remove decision
        del readable_state["DECISION"]

        discretised_variables = ["Mutual Gaze","Engagement Value","Pose Estimation Confidence","Engagement Level Confidence","Motion Confidence","Group Confidence","Distance"]
        for key in readable_state:
            for variable in readable_state[key]:
                var_text = ""
                if isinstance(readable_state[key][variable],float):
                    var_text += str(round(readable_state[key][variable],3))
                elif variable == "Engagement Level":
                    var_text = MessageHelper.engagement_level_names[readable_state[key][variable]]
                elif variable == "Motion":
                    var_text = MessageHelper.motion_activity_names[readable_state[key][variable]]
                else:
                    var_text += str(readable_state[key][variable])

                if variable in discretised_variables:
                    disc_var = discrete_state[key][variable]
                    if variable != "Distance":
                        disc_var = round(disc_var/DecisionState.max_float,3)
                    var_text += " ({})".format(disc_var)

                readable_state[key][variable] = var_text
        return readable_state

    
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







