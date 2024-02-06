import rospy

from engage.decision_maker.decision_maker import DecisionState,Decision
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


    def message(self,decision:Decision,msg):
        decision_state = msg()
        decision_state.header.stamp = self.time

        empty_velocity = Twist()
        empty_velocity.linear.x = 0
        empty_velocity.linear.y = 0
        empty_velocity.linear.z = 0
        empty_velocity.angular.x = 0
        empty_velocity.angular.y = 0
        empty_velocity.angular.z = 0

        decision_state.bodies = self.bodies
        decision_state.groups = [self.groups[body] for body in self.bodies]
        decision_state.velocities = [self.velocities[body] if self.velocities[body] is not None else empty_velocity for body in self.bodies]
        decision_state.distances = [self.distances[body] if self.distances[body] is not None else 0 for body in self.bodies]
        decision_state.mutual_gazes = [self.mutual_gazes[body] if self.mutual_gazes[body] is not None else 0 for body in self.bodies]
        decision_state.engagement_values = [self.engagement_values[body] if self.engagement_values[body] is not None else 0 for body in self.bodies]
        decision_state.pose_estimation_confidences = [self.pose_confidences[body] if self.pose_confidences[body] is not None else 0 for body in self.bodies]
        decision_state.engagement_levels = [self.engagement_levels[body] for body in self.bodies]
        decision_state.engagement_level_confidences = [self.engagement_level_confidences[body] for body in self.bodies]
        decision_state.motion_activities = [self.motions[body] for body in self.bodies]
        decision_state.motion_activity_confidences = [self.motion_confidences[body] for body in self.bodies]
        decision_state.groups = [self.in_group[body] for body in self.bodies]
        decision_state.group_with_robot = [self.group_with_robot[body] for body in self.bodies]
        decision_state.group_confidences = [self.group_confidences[body] if self.group_confidences[body] is not None else 0 for body in self.bodies]
        decision_state.robot_group = self.robot_in_group
        decision_state.robot_group_confidence = self.robot_group_confidence if self.robot_group_confidence is not None else 0
        decision_state.waiting = self.waiting

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
        
        states["ROBOT"]["Group"].append(msg.robot_group)
        states["ROBOT"]["Group Confidence"].append(msg.robot_group_confidence)
        state_times["ROBOT"].append(msg.header.stamp.to_sec())
        state_bodies.append(msg.bodies)

        # TODO: Decision
        states = decision.update_state_decision(states,msg.decision)

        return states,state_bodies,state_times
    
    @staticmethod
    def create_publisher(msg,topic="hri_engage/decision_states",queue_size=1):
        return rospy.Publisher(topic,msg,queue_size=queue_size)