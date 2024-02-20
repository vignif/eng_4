import numpy as np

from engage.explanation.explain_test.explainability_test import ExplainabilityTest

class EngageStateExplainabilityTest(ExplainabilityTest):
    def __init__(self,explanations,group,var_nums,ignore_uninteresting=True,restricted_actions=None) -> None:
        '''
        Group 0 - control
        Group 1 - explanation but not counterfactial
        Group 2 - explanation and counterfactual
        '''
        self.group = group

        if len(explanations) == 0:
            raise Exception("No explanations")
        
        # Pick an appropriate explanation based on the variable
        exp_vars = [(i,exp.variables[0]) for i,exp in zip(range(len(explanations)),explanations) if self.valid_variable(exp.variables[0],ignore_uninteresting)]
        exp_counts = np.array([var_nums[exp_var[1].split("_")[1]] for exp_var in exp_vars])
        min_exps = np.where(exp_counts == exp_counts.min())[0]
        picked_index = np.random.choice(min_exps)

        self.explanation = explanations[exp_vars[picked_index][0]]

        # Update var num dict for future explanations
        exp_split = exp_vars[picked_index][1].split("_")
        self.exp_var = exp_split[1]
        self.exp_target = exp_split[0] if exp_split[0] not in ["GENERAL","ROBOT"] else ""
        var_nums[self.exp_var] += 1

        # Generate explanation texts
        self.exp_counterfactual = ""
        if group == 0:
            self.exp_reason = self.explanation.true_action_text() + "."
        elif group == 1:
            self.exp_reason,_ = self.explanation.present_explanation()
        elif group == 2:
            self.exp_reason,self.exp_counterfactual = self.explanation.present_explanation()

        # Generate followup questions
        self.question_context,question_texts,outcomes = self.explanation.follow_up_new_person()
        q_index = np.random.choice(range(len(question_texts)))

        self.question_text = question_texts[q_index]

        # Generate follow up answers
        right_answer = outcomes[q_index]

        # Generate distractors
        self.answer_texts,self.right_answer_index,self.answer_components = self.explanation.generate_distractors(right_answer,restricted_actions=restricted_actions)

    def valid_variable(self,var,ignore_uninteresting):
        varsplit = var.split("_")
        if ignore_uninteresting:
            if varsplit[0] in ["GENERAL","ROBOT"]:
                return False
        return True