import rospy
import copy

from engage.decision_maker.decision_maker import DecisionState,Decision
from engage.msg import DecisionState as DecisionStateMSG
from geometry_msgs.msg import Twist

class EngageState(DecisionState):

    engagement_level_names = ["UNKNOWN","DISENGAGED","ENGAGING","ENGAGED","DISENGAGING"]
    motion_names = ["NOTHING","WALKING_AWAY","WALKING_TOWARDS","WALKING_PAST"]

    def __init__(self) -> None:
        super().__init__()

    def node_to_state(
            self,
            time,
            body_dict,
            group_dict,
            group_confidences,
            distances,
            engagements,
            mutual_gazes,
            pose_confidences,
            waiting,
    ):
        # Create from the various dicts and lists maintained by the decision node
        self.time = time
        self.bodies = list(body_dict.keys())
        self.groups = group_dict.copy()
        self.group_confidences = group_confidences.copy()
        self.engagement_values = engagements.copy()
        self.distances = distances.copy()
        self.mutual_gazes = mutual_gazes.copy()
        self.pose_confidences = pose_confidences.copy()

        self.motions = {}
        self.motion_confidences = {}
        self.engagement_levels = {}
        self.engagement_level_confidences = {}
        self.velocities = {}
        for body in body_dict:
            self.motions[body] = body_dict[body].activity
            self.motion_confidences[body] = body_dict[body].activity_confidence
            self.engagement_levels[body] = body_dict[body].engagement_level
            self.engagement_level_confidences[body] = body_dict[body].engagement_level_confidence
            self.velocities[body] = body_dict[body].velocity

        self.in_group = {}
        self.group_with_robot = {}
        self.robot_in_group = self.check_group("ROBOT")
        self.robot_group_confidence = self.group_confidences["ROBOT"]
        self.robot_group_members = []
        for body in self.bodies:
            self.in_group[body] = self.check_group(body)
            self.group_with_robot[body] = self.robot_in_group and (self.groups[body] == self.groups["ROBOT"])
            if self.group_with_robot[body]:
                self.robot_group_members.append(body)

        self.waiting = waiting

    def dict_to_state(
          self,
          state,
          bodies  
    ):
        self.bodies = bodies

        self.group_confidences = {}
        self.engagement_values = {}
        self.distances = {}
        self.mutual_gazes = {}
        self.pose_confidences = {}
        self.motions = {}
        self.motion_confidences = {}
        self.engagement_levels = {}
        self.engagement_level_confidences = {}
        self.in_group = {}
        self.group_with_robot = {}
        self.robot_group_members = []
        # TODO: Add velocity
        self.velocities = {}

        for key in state:
            if key == "GENERAL":
                self.waiting = state[key]["Waiting"]
            elif key == "ROBOT":
                self.robot_in_group = state[key]["Group"]
                self.robot_group_confidence = state[key]["Group Confidence"]
            else:
                self.group_confidences[key] = state[key]["Group Confidence"]
                self.engagement_values[key] = state[key]["Engagement Value"]
                self.distances[key] = state[key]["Distance"]
                self.mutual_gazes[key] = state[key]["Mutual Gaze"]
                self.pose_confidences[key] = state[key]["Pose Estimation Confidence"]
                self.motions[key] = state[key]["Motion"]
                self.motion_confidences[key] = state[key]["Motion Confidence"]
                self.engagement_levels[key] = state[key]["Engagement Level"]
                self.engagement_level_confidences[key] = state[key]["Engagement Level Confidence"]
                self.in_group[key] = state[key]["Group"]
                self.group_with_robot[key] = state[key]["Group with Robot"]
                if state[key]["Group with Robot"]:
                    self.robot_group_members.append(key)


    def message(self,decision:Decision,msg,dm_name,exclude_None_bodies=True):
        decision_state = msg()
        decision_state.header.stamp = self.time
        decision_state.decision_maker = dm_name

        state_msg = DecisionStateMSG()

        empty_velocity = Twist()
        empty_velocity.linear.x = 0
        empty_velocity.linear.y = 0
        empty_velocity.linear.z = 0
        empty_velocity.angular.x = 0
        empty_velocity.angular.y = 0
        empty_velocity.angular.z = 0

        if exclude_None_bodies:
            new_bodies = [body for body in self.bodies if self.distances[body] is not None]
        else:
            new_bodies = self.bodies

        state_msg.bodies = new_bodies
        state_msg.groups = [self.groups[body] for body in new_bodies]
        state_msg.velocities = [self.velocities[body] if self.velocities[body] is not None else empty_velocity for body in new_bodies]
        state_msg.distances = [self.distances[body] if self.distances[body] is not None else 0 for body in new_bodies]
        state_msg.mutual_gazes = [self.mutual_gazes[body] if self.mutual_gazes[body] is not None else 0 for body in new_bodies]
        state_msg.engagement_values = [self.engagement_values[body] if self.engagement_values[body] is not None else 0 for body in new_bodies]
        state_msg.pose_estimation_confidences = [self.pose_confidences[body] if self.pose_confidences[body] is not None else 0 for body in new_bodies]
        state_msg.engagement_levels = [self.engagement_levels[body] for body in new_bodies]
        state_msg.engagement_level_confidences = [self.engagement_level_confidences[body] for body in new_bodies]
        state_msg.motion_activities = [self.motions[body] for body in new_bodies]
        state_msg.motion_activity_confidences = [self.motion_confidences[body] for body in new_bodies]
        state_msg.groups = [self.in_group[body] for body in new_bodies]
        state_msg.group_with_robot = [self.group_with_robot[body] for body in new_bodies]
        state_msg.group_confidences = [self.group_confidences[body] if self.group_confidences[body] is not None else 0 for body in new_bodies]
        state_msg.robot_group = self.robot_in_group
        state_msg.robot_group_confidence = self.robot_group_confidence if self.robot_group_confidence is not None else 0
        state_msg.waiting = self.waiting

        decision_state.state = state_msg
        decision_state.decision = decision.message(self.time)

        return decision_state
    
    @staticmethod
    def create_state_dicts(decision:Decision):
        states = {}
        state_bodies = []
        state_times = {}
        states["ROBOT"] = {"Group":[],"Group Confidence":[]}
        states["GENERAL"] = {"Waiting":[]}
        states["DECISION"] = {}
        for component in decision.component_names:
            states["DECISION"][component] = []
        state_times["ROBOT"] = []

        return states,state_bodies,state_times

    @staticmethod
    def update_state_dicts_from_msg(states,state_bodies,state_times,msg,decision:Decision):
        for i in range(len(msg.state.bodies)):
            body = msg.state.bodies[i]

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
            states[body]["Distance"].append(msg.state.distances[i])
            states[body]["Mutual Gaze"].append(msg.state.mutual_gazes[i])
            states[body]["Engagement Value"].append(msg.state.engagement_values[i])
            states[body]["Pose Estimation Confidence"].append(msg.state.pose_estimation_confidences[i])
            states[body]["Engagement Level"].append(msg.state.engagement_levels[i])
            states[body]["Engagement Level Confidence"].append(msg.state.engagement_level_confidences[i])
            states[body]["Motion"].append(msg.state.motion_activities[i])
            states[body]["Motion Confidence"].append(msg.state.motion_activity_confidences[i])
            states[body]["Group"].append(msg.state.groups[i])
            states[body]["Group Confidence"].append(msg.state.group_confidences[i])
            states[body]["Group with Robot"].append(msg.state.group_with_robot[i])

        states["GENERAL"]["Waiting"].append(msg.state.waiting)
        
        states["ROBOT"]["Group"].append(msg.state.robot_group)
        states["ROBOT"]["Group Confidence"].append(msg.state.robot_group_confidence)
        state_times["ROBOT"].append(msg.header.stamp.to_sec())
        state_bodies.append(msg.state.bodies)

        states = decision.update_state_decision(states,msg.decision)

        return states,state_bodies,state_times,msg.decision_maker
    
    @staticmethod
    def single_state_from_msg(msg):
        state = {}
        state_bodies = msg.state.bodies

        state["GENERAL"] = {}
        state["ROBOT"] = {}

        for i in range(len(msg.state.bodies)):
            body = state_bodies[i]
            state[body] = {}
            state[body]["Distance"] = msg.state.distances[i]
            state[body]["Mutual Gaze"] = msg.state.mutual_gazes[i]
            state[body]["Engagement Value"] = msg.state.engagement_values[i]
            state[body]["Pose Estimation Confidence"] = msg.state.pose_estimation_confidences[i]
            state[body]["Engagement Level"] = msg.state.engagement_levels[i]
            state[body]["Engagement Level Confidence"] = msg.state.engagement_level_confidences[i]
            state[body]["Motion"] = msg.state.motion_activities[i]
            state[body]["Motion Confidence"] = msg.state.motion_activity_confidences[i]
            state[body]["Group"] = msg.state.groups[i]
            state[body]["Group Confidence"] = msg.state.group_confidences[i]
            state[body]["Group with Robot"] = msg.state.group_with_robot[i]

        state["GENERAL"]["Waiting"] = msg.state.waiting
        
        state["ROBOT"]["Group"] = msg.state.robot_group
        state["ROBOT"]["Group Confidence"] = msg.state.robot_group_confidence

        return state,state_bodies


    
    @staticmethod
    def create_publisher(msg,topic="hri_engage/decision_states",queue_size=1):
        return rospy.Publisher(topic,msg,queue_size=queue_size)
    
    @staticmethod
    def discretise(state,bodies):
        discrete_state = copy.deepcopy(state)

        discrete_state["ROBOT"]["Group Confidence"] = DecisionState.float_bucket(discrete_state["ROBOT"]["Group Confidence"])

        body_floats = ["Mutual Gaze","Engagement Value","Pose Estimation Confidence","Engagement Level Confidence","Motion Confidence","Group Confidence"]

        for body in bodies:
            discrete_state[body]["Distance"] = DecisionState.distance_bucket(discrete_state[body]["Distance"])
            for bf in body_floats:
                discrete_state[body][bf] = DecisionState.float_bucket(discrete_state[body][bf])
        return discrete_state