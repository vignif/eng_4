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
        self.max_pec = self.discrete_score(1)

    '''
    DECISION
    '''

    decision = HeuristicDecision
        
    def update_last_decision_time(self,decision:HeuristicDecision,time):
        if decision.action != HeuristicDecisionMSG.NOTHING and decision.action != HeuristicDecisionMSG.WAIT:
            self.last_decision_time = time

    def decide_simple(self,state,score_threshold=0.1):
        target = None
        action = None

        if state.waiting:
            action = HeuristicDecisionMSG.WAIT
        elif len(state.bodies) == 0:
            # Nobody to interact with
            action = HeuristicDecisionMSG.NOTHING
        else:
            scores = self.simple_scores(state)
            best_people = [kv[0] for kv in scores.items() if kv[1] == max(scores.values())]
            if len(best_people) == 0:
                action = action = HeuristicDecisionMSG.NOTHING
            else:
                potential_target = sorted(best_people)[0]
                if scores[potential_target] <= score_threshold:
                    action = HeuristicDecisionMSG.ELICIT_GENERAL
                else:
                    action = HeuristicDecisionMSG.ELICIT_TARGET
                    target = potential_target
        return HeuristicDecision(action,target)

    def decide(self,state:EngageState,discretise=True):
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
            '''
            print("Bodies: ",state.bodies)
            print("EVs: ",state.engagement_values)
            print("Ds: ",state.distances)
            print("MGs: ",state.mutual_gazes)
            '''

            #scores = {k: self.discrete_score(v) for k, v in state.engagement_values.items() if v is not None}
            scores = self.calculate_scores(state)
            best_people = [kv[0] for kv in scores.items() if kv[1] == max(scores.values())]
            if len(best_people) == 0:
                action = action = HeuristicDecisionMSG.NOTHING
            else:
                potential_target = sorted(best_people)[0]
                if scores[potential_target] == 0:
                    action = HeuristicDecisionMSG.ELICIT_GENERAL
                else:
                    action = HeuristicDecisionMSG.ELICIT_TARGET
                    target = potential_target

            '''
            print("Scores: ",scores)
            print("Best People: ",best_people)
            print("{}_{}".format(action,target))
            '''

        return HeuristicDecision(action,target)
    
    def simple_scores(self,state):
        scores = {}
        for body in state.bodies:
            ev = state.engagement_values[body]
            pec = state.pose_confidences[body]
            scores[body] = ev*pec
        return scores
    
    def calculate_scores(self,state:EngageState):
        scores = {}
        for body in state.bodies:
            ev = state.engagement_values[body]
            if ev is None:
                scores[body] = -9999999
                continue
            dev = self.discrete_score(ev)
            pec = state.pose_confidences[body]
            if pec is None:
                scores[body] = 0
                continue
            dpec = self.discrete_score(pec)/self.max_pec
            scores[body] = dpec*dev
        return scores
                
    
    def discrete_score(self,score):
        # NOTE: This is here so that discretisation in the explanations doesn't lead to weird results
        # e.g. If A has score=0.5 and B has a score of 0.51, undiscretised chooses B but discretised has them both as 0.5 and picks A, resulting in no changes producing a new result
        return EngageState.float_bucket(score)