import rospy

from engage.msg import HeuristicDecision as HeuristicDecisionMSG,EngagementLevel,MotionActivity
from engage.decision_maker.engage_state import EngageState
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.decision_maker import DecisionMaker


class HeuristicDecisionMaker(DecisionMaker):
    def __init__(self,wait_time=5,score_threshold=0,stochastic=False,**kwargs):
        self.stochastic = stochastic
        self.score_threshold = score_threshold
        self.wait_time = rospy.Duration(wait_time)

        self.last_decision_time = None

    '''
    DECISION
    '''

    decision = HeuristicDecision
        
    def update_last_decision_time(self,decision:HeuristicDecision,time):
        if decision.action != HeuristicDecisionMSG.NOTHING and decision.action != HeuristicDecisionMSG.WAIT:
            self.last_decision_time = time

    def decide(self,state:EngageState):
        target = None
        action = None

        if state.waiting:
            # Waiting for an action to execute
            action = HeuristicDecisionMSG.WAIT
        elif len(state.bodies) == 0:
            # Nobody to interact with
            action = HeuristicDecisionMSG.NOTHING
        elif state.robot_in_group and len(state.robot_group_members) != 0: # The check for robot group members is there to prevent bugs when intervening on the robot group variable # TODO fix this
            # Robot in a group
            all_engaged = True
            for body in state.robot_group_members:
                if not (state.engagement_levels[body] == EngagementLevel.ENGAGED or state.engagement_levels[body] == EngagementLevel.ENGAGING):
                    # Somebody not engaged, try to recapture them
                    all_engaged = False
                    action = HeuristicDecisionMSG.RECAPTURE
                    target = body
            if all_engaged:
                # Everyone engaged, maintain interaction
                action = HeuristicDecisionMSG.MAINTAIN
                target = state.robot_group_members[0]
        else:
            # Robot not in a group, must attract someone's attention

            # Calculate a score for everyone
            scores = self.calculate_scores(state)

            # Get the people who maximise the score
            best_people = [kv[0] for kv in scores.items() if kv[1] == max(scores.values())]

            valid_people = [body for body in best_people if scores[body] > self.score_threshold]

            if len(valid_people) == 0:
                action = HeuristicDecisionMSG.ELICIT_GENERAL
            else:
                action = HeuristicDecisionMSG.ELICIT_TARGET
                target = valid_people[0]

        return HeuristicDecision(action,target)
    
    def calculate_scores(self,
                         state:EngageState,
                         gaze_thresh=0.75,
                         close_dist=1,
                         mid_dist=3,
                         group_weight=1,
                         engagement_weight=1,
                         activity_weight=1,
                         distance_weight=1
                        ):
        scores = {}
        for body in state.bodies:
            score = 0


            # Check for Nones
            if state.mutual_gazes[body] is None or state.distances[body] is None or state.engagement_values[body] is None or state.pose_confidences[body] is None:
                scores[body] = -100000
                continue
            
            # Group
            if state.in_group[body]:
                # Person in a group...
                if state.mutual_gazes[body]>gaze_thresh:
                    # ...and looking at robot
                    score += group_weight*state.group_confidences[body]*state.pose_confidences[body]
                else:
                    # ...and looking away
                    score -= group_weight*state.group_confidences[body]*state.pose_confidences[body]
            
            if state.engagement_levels[body]==EngagementLevel.ENGAGED or state.engagement_levels[body]==EngagementLevel.ENGAGING:
                # Person is engaged or engaging
                score += engagement_weight*state.engagement_level_confidences[body]

            
            if state.motions[body]==MotionActivity.WALKING_TOWARDS:
                # person is walking towards the robot
                score += activity_weight*state.motion_confidences[body]
            elif state.motions[body]==MotionActivity.WALKING_AWAY:
                # person is walking away from the robot
                score -= activity_weight*state.motion_confidences[body]

            # Score based on distance
            if state.distances[body] < mid_dist:
                score += distance_weight*(1-(state.distances[body]/mid_dist))*state.pose_confidences[body]
                if state.distances[body] < close_dist:
                    score += distance_weight*(1-(state.distances[body]/close_dist))*state.pose_confidences[body]
            else:
                score -= distance_weight*state.pose_confidences[body]
            
            scores[body] = score
        return scores