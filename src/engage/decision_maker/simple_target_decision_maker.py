import rospy

from engage.msg import HeuristicDecision as HeuristicDecisionMSG,EngagementLevel,MotionActivity
from engage.decision_maker.engage_state import EngageState
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.decision_maker import DecisionMaker


class SimpleTargetDecisionMaker(DecisionMaker):
    def __init__(self,wait_time=5,stochastic=False,**kwargs):
        self.stochastic = stochastic
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
        else:
            # Find the bodies that maximise the engagement score
            scores = {k: v for k, v in state.engagement_values.items() if v is not None}
            best_people = [kv[0] for kv in scores.items() if kv[1] == max(scores.values())]
            if len(best_people) == 0:
                action = action = HeuristicDecisionMSG.NOTHING
            else:
                action = HeuristicDecisionMSG.ELICIT_TARGET
                target = best_people[0]

        return HeuristicDecision(action,target)