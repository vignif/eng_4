import copy

from engage.decision_maker.engage_state import EngageState
from engage.explanation.engage_state_observation import EngageStateObservation
from engage.decision_maker.decision_manager import DecisionManager
from engage.msg import HeuristicDecision as HeuristicDecisionMSG
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.explanation.counterfactual_explainer import CounterfactualExplainer
from engage.explanation.heuristic_explainer import HeuristicCounterfactual

class HRIExplainer:
    def __init__(self,decision_maker):
        self.decision_maker = decision_maker

    def setup_explanation(self,decision_state_msg,query=None,decision_maker="heuristic"):
        self.state,self.bodies = EngageState.single_state_from_msg(decision_state_msg)
        self.discrete_state = EngageState.discretise(self.state,self.bodies)
        self.true_observation = EngageStateObservation(self.discrete_state,self.bodies)
        if query is not None:
            self.query = query
        else:
            self.query = DecisionManager.decision_queries[decision_maker]()
        self.true_outcome = DecisionManager.decision_outcomes[decision_maker](decision_state_msg.decision)

        return self.validate_query()

    def validate_query(self):
        text_explanation = None

        if not self.query.is_none():
            if self.query == self.true_outcome:
                text_explanation = "Your query is exactly the decision the robot made"
            elif self.query.target is not None:
                if self.query.target == "ROBOT":
                    text_explanation = "The robot cannot target itself"
                elif self.query.target not in self.bodies:
                    text_explanation = "The body {} is not recognised at the time of the decision".format(self.query.target)
                elif self.query.action in [HeuristicDecisionMSG.NOTHING,HeuristicDecisionMSG.WAIT,HeuristicDecisionMSG.ELICIT_GENERAL]:
                    text_explanation = "The action {} cannot take a target".format(HeuristicDecision.action_names[self.query.action])

        return text_explanation is None,text_explanation
    
    def explain(self,display=True,max_depth=2):
        self.display = display
        if self.display:
            print("===Observation===")
            print(self.true_observation)
            print("===Decision===")
            print(self.true_outcome)
            print("===Query===")
            print(self.query)

        cfx = CounterfactualExplainer(self.true_observation,self.true_outcome,HeuristicCounterfactual,self.decision_maker)
        explanations = cfx.explain(self.query,max_depth)
        print(explanations)