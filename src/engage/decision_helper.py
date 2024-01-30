from engage.msg import StateDecision

class DecisionState:
    max_float = 3

    def __init__(self,
                 time=None,
                 body_dict=None,
                 group_dict=None,
                 group_confidences=None,
                 distances=None,
                 engagements=None,
                 mutual_gazes=None,
                 pose_confidences=None,
                 waiting=None,
                 from_decision_node=True,
                 state=None,
                 bodies=None,
                 ):
        if from_decision_node:
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
            for body in body_dict:
                self.motions[body] = body_dict[body].activity
                self.motion_confidences[body] = body_dict[body].activity_confidence
                self.engagement_levels[body] = body_dict[body].engagement_level
                self.engagement_level_confidences[body] = body_dict[body].engagement_level_confidence

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
        else:
            # Create from a state
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


    def message(self,action,target):
        decision_state = StateDecision()
        decision_state.header.stamp = self.time

        decision_state.bodies = self.bodies
        decision_state.groups = [self.groups[body] for body in self.bodies]
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

        decision_state.decision_action = action
        decision_state.decision_target = target if target is not None else ""

        return decision_state


    
    def check_group(self,id):
        # True if id's group > 1 person
        same_group = [k for k,v in self.groups.items() if v == self.groups[id]]
        return len(same_group)>1

    @staticmethod
    def float_bucket(value):
        if value < 0.25:
            return 0
        elif value < 0.5:
            return 1
        elif value < 0.75:
            return 2
        else:
            return 3

    @staticmethod
    def distance_bucket(distance):
        if distance <= 3:
            discretised_distance = round(distance * 2) / 2
        else:
            discretised_distance = round(distance)

        if discretised_distance == 0.0:
            discretised_distance = 0.1
        elif discretised_distance > 8:
            discretised_distance = 9

        return discretised_distance