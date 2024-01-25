from engage.msg import StateDecision

class DecisionState:
    def __init__(self,
                 time,
                 body_dict,
                 group_dict,
                 group_confidences,
                 distances,
                 engagements,
                 mutual_gazes,
                 pose_confidences,
                 waiting
                 ):
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
            if self.group_with_robot:
                self.robot_group_members.append(body)

        self.waiting = waiting

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