from engage.explanation.explain_test.engage_state_explainability_test import EngageStateExplainabilityTest
from engage.msg import HeuristicDecision as HeuristicDecisionMSG
from explanation_msgs.msg import Explainability

class HeuristicExplainabilityTest(EngageStateExplainabilityTest):
    def __init__(self, 
                 explanations, 
                 group, 
                 var_nums, 
                 ignore_uninteresting=True,
                 names=None,
                 restricted_actions=[HeuristicDecisionMSG.ELICIT_GENERAL,HeuristicDecisionMSG.ELICIT_TARGET]
                 ) -> None:
        super().__init__(explanations, group, var_nums, ignore_uninteresting,names,restricted_actions)

        # Heuristic stuff
        self.answer_actions = self.answer_components["Actions"]
        self.answer_targets = self.answer_components["Targets"]

    def to_message(self,image):
        msg = Explainability()

        msg.group = self.group
        msg.image = image
        msg.explanation = self.exp_reason
        msg.counterfactual = self.exp_counterfactual
        msg.question_context = self.question_context
        msg.question_text = self.question_text
        msg.answer_list = self.answer_texts
        msg.correct_answer = self.right_answer_index
        msg.true_decision_action = self.explanation.true_outcome.string_action()
        msg.true_decision_target = self.explanation.true_outcome.string_target()
        msg.explanation_variable = self.exp_var
        msg.explanation_target = self.exp_target
        msg.answer_actions = self.answer_actions
        msg.answer_targets = self.answer_targets

        return msg
    