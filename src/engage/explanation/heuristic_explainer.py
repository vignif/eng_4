import copy
import numpy as np

from engage.explanation.counterfactual_explainer import Outcome,Query,Counterfactual,Explanation
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.engage_state import EngageState
from engage.explanation.engage_state_observation import EngageStateObservation
from engage.msg import HeuristicDecision as HeuristicDecisionMSG, MotionActivity, EngagementLevel
from engage.explanation.language.heuristic_text_english import HeuristicTextEnglish
from engage.explanation.language.heuristic_text_catalan import HeuristicTextCatalan
      
class HeuristicCounterfactual(Counterfactual):
    def __init__(self, decision_maker, intervention_order=[], interventions={}, changes={}):
        super().__init__(decision_maker, intervention_order, interventions, changes)

    def copy(self):
        return HeuristicCounterfactual(self.decision_maker,intervention_order=self.intervention_order,interventions=copy.deepcopy(self.interventions),changes=copy.deepcopy(self.changes))
    
    def outcome(self,observation,intervention_order=None,interventions=None):
        if intervention_order is None:
            intervention_order = self.intervention_order
        if interventions is None:
            interventions = self.interventions

        '''
        print("\n\n")
        print("Original State: ",observation.state)
        print("-")
        print("Interventions: ",interventions)
        '''
        

        changes = self.apply_interventions(observation,intervention_order,interventions)

        state = self.full_state(observation,changes)

        continuous_state = self.undsicretise_state(state,observation)

        '''
        print("Changes: ",changes)
        print("New State: ",state)
        '''
        
        decision_state = EngageState()
        decision_state.dict_to_state(continuous_state,observation.bodies)

        dm_outcome = self.decision_maker.decide(decision_state)

        '''
        print("Decision State: ",decision_state.group_with_robot)
        print("Outcome: ",dm_outcome.action,dm_outcome.target)
        '''

        return HeuristicOutcome(dm_outcome)
    
    def undsicretise_state(self,state,observation:EngageStateObservation):
        new_state = copy.deepcopy(state)
        for key in state:
            for var in state[key]:
                if observation.variable_categories[var] == "Continuous" and var != "Distance":
                    new_state[key][var] = state[key][var]/max(observation.variable_cardinalities[var])
        return new_state
        

class HeuristicQuery(Query):
    def __init__(self,action=None,target=None):
        self.action = action
        self.target = target
        if self.target == "":
            self.target = None

    def is_none(self):
        return self.target is None and self.action is None
    
    def __str__(self):
        if self.action is not None:
            return "<{},{}>".format(HeuristicDecision.action_names[self.action],self.target)
        return "<{},{}>".format(self.action,self.target)
        
class HeuristicOutcome(Outcome):
    def __init__(self,decision:HeuristicDecision):
        self.action = decision.action
        self.target = decision.target
        if self.target == "":
            self.target = None

    def __eq__(self, other):
        if type(other) is type(self):
            return self.__dict__ == other.__dict__
        return False
    
    def __str__(self):
        return "<{},{}>".format(HeuristicDecision.action_names[self.action],self.target)
    
    def string_action(self):
        return HeuristicDecision.action_names[self.action]
    
    def string_target(self):
        return self.target if self.target is not None else "None"
    
    def valid_outcome(self,outcome,query):
        
        if query.is_none():
            # Valid if different from the true outcome
            return outcome != self
        else:
            if query.target is None:
                # We care about the decision, not the target
                return outcome.action == query.action
            elif query.action is None:
                # We care about the target, not the decision
                return outcome.target == query.target
            else:
                # We care about both
                return outcome == query
            
    def is_none(self):
        return self.target is None and self.action is None
    
class HeuristicExplanation(Explanation):
    def __init__(self,true_outcome:HeuristicOutcome,true_observation:EngageStateObservation,query:HeuristicQuery,counterfactual:HeuristicCounterfactual):
        self.true_observation = true_observation
        self.true_outcome = true_outcome
        self.query = query
        self.counterfactual = counterfactual
        self.bodies = self.get_bodies()
        self.names = {}
        super().__init__()

    def get_bodies(self):
        return [key for key in list(self.true_observation.state.keys()) if key not in ["GENERAL","ROBOT"]]

    def make_critical_explanation(self,variable,values,outcomes):
        self.num_vars = 1
        self.variables = [variable]
        var_split = variable.split("_")
        self.var_cats = {variable:var_split[0]}
        self.var_names = {variable:var_split[1]}
        self.true_values = {variable:self.true_observation.value_of_variable_in_assignment(variable,self.true_observation.state)}
        self.explanation_values = {variable:values}
        self.outcomes = {variable:outcomes}
        
    def present_explanation(self):
        var = self.variables[0]
        subject =  self.var_cats[var]
        true_val = self.true_values[self.variables[0]]
        reason = self.text_generator.construct_reason_text(self.true_outcome,var,self.var_names[var],self.var_cats,subject,self.explanation_values,true_val,self.true_observation,self.person_name)
        counterfactual = self.counterfactual_explanation_component()
        return reason,counterfactual
    
    def future_outcome_text(self,outcome):
        return self.text_generator.outcome_text_future(outcome,self.person_name)
    
    def outcome_text(self,outcome):
        return self.text_generator.outcome_text_past(outcome,self.person_name)

    def true_action_text(self):
        return self.outcome_text(self.true_outcome)
    
    def counterfactual_explanation_component(self):
        if self.num_vars == 1:
            return self.counterfactual_component_var_single_var(self.variables[0])
        raise NotImplementedError
    
    def counterfactual_component_var_single_var(self,var):
        # Cluster outcomes
        outcome_clusters = {}
        outcome_objects = {}
        for i in range(len(self.explanation_values[var])):
            if str(self.outcomes[var][i]) not in outcome_clusters:
                outcome_clusters[str(self.outcomes[var][i])] = []
                outcome_objects[str(self.outcomes[var][i])] = self.outcomes[var][i]
            outcome_clusters[str(self.outcomes[var][i])].append(self.explanation_values[var][i])

        # Explain each counterfactual outcome
        counterfactual_components = []
        for outcome in outcome_clusters:
            foil_text = self.foil_text_single_var(var,outcome_clusters[outcome])
            counterfactual_decision_text = self.text_generator.outcome_text_conditional(outcome_objects[outcome],self.person_name)
            counterfactual_components.append(self.text_generator.construct_counterfactual_text(foil_text,counterfactual_decision_text))

        # Concantenate
        counterfactual_text = ""
        for comp in counterfactual_components:
            counterfactual_text += comp
        return counterfactual_text[1:]


    def foil_text_single_var(self,var,values):
        var_name = self.var_names[var]
        subject =  self.person_name(self.var_cats[var])
        cards = self.true_observation.variable_cardinalities[var_name]

        return self.text_generator.foil_text(var,var_name,self.var_cats,values,subject,self.true_values,self.true_observation,cards)


    def person_name(self,target_name):
        if target_name in self.names:
            return self.names[target_name]
        else:
            return target_name
    
    def update_names(self,names):
        if names is None:
            self.names = {"NEWPERSON":"Bob"}
        else:
            self.names = copy.deepcopy(names)
            self.names["NEWPERSON"] = "Bob"

    def set_language(self,language:str):
        if language.lower() in ["english","en_gb"]:
            self.text_generator = HeuristicTextEnglish()
        elif language.lower() in ["catalan","ca_es"]:
            self.text_generator = HeuristicTextCatalan()
        else:
            raise Exception("Language {} not recognised".format(language))


    
    '''
    FOLLOW UP
    '''
    
    def follow_up_question(self):
        if len(self.variables) == 1:
            follow_up_vars = []
            follow_up_questions = []
            if self.var_cats[self.variables[0]] not in ["ROBOT","GENERAL"]:
                for var_cat in self.true_observation.state:
                    if var_cat in ["ROBOT","GENERAL"]:
                        continue
                    if self.var_names[self.variables[0]] in self.true_observation.state[var_cat]:
                        follow_up_vars.append((var_cat,self.var_names[self.variables[0]]))
                
            for var in follow_up_vars:
                var_name = "{}_{}".format(var[0],var[1])
                for val in self.true_observation.variable_cardinalities[var[1]]:
                    if val == self.true_observation.state[var[0]][var[1]]:
                        continue
                    if var_name == self.variables[0]:
                        # Don't use the same body as the counterfactual
                        continue
                    num_val = val
                    if self.true_observation.variable_categories[var[1]] == "Continuous" and var[1]!="Distance":
                        num_val = val/max(self.true_observation.variable_cardinalities[var[1]])
                    follow_up_questions.append((var,num_val))
        
            # Display followups
            questions = []
            right_answers = []
            distractors = []
            for fuq in follow_up_questions:
                # Get follow up question
                question_statement,question_context = self.question_text(fuq[0][0],fuq[0][1],fuq[1])
                question_context = self.text_generator.question_context(question_context)
                question_text = self.text_generator.question_text(question_statement)
                fuq_text = question_context + " " + question_text
                questions.append(fuq_text)
                print("\t - {}".format(fuq_text))
                #print("\t - {},{}".format(fuq[1],self.true_observation.state[fuq[0][0]][fuq[0][1]]))
                # Get right answer
                val = fuq[1]
                if self.true_observation.variable_categories[fuq[0][1]] == "Continuous" and fuq[0][1] != "Distance":
                    val = val*max(self.true_observation.variable_cardinalities[fuq[0][1]])
                intervention = {fuq[0][0]:{fuq[0][1]:val}}
                var_full_name = "{}_{}".format(fuq[0][0],fuq[0][1])
                right_decision = self.counterfactual.outcome(self.true_observation,self.counterfactual.intervention_order+[var_full_name],intervention)
                print("\t\t - {}".format(right_decision))
                right_answers.append(right_decision)
                # Get Distractors
                distractors.append(self.get_distractors(right_decision))
                for distractor in distractors[-1]:
                    print("\t\t\t . {}".format(HeuristicOutcome(HeuristicDecision(distractor[0],distractor[1]))))
                    pass
        else:
            raise NotImplementedError
        
        return questions,right_answers,distractors
        
    def question_text(self,var_cat,var_name,value,prelude=None):
        subject = self.person_name(var_cat)
        question_statement = self.text_generator.statement_text_conditional(var_cat,var_name,value,subject,self.true_observation)

        num_val = self.true_observation.state[var_cat][var_name]
        if self.true_observation.variable_categories[var_name] == "Continuous" and var_name!="Distance":
            num_val = self.true_observation.state[var_cat][var_name]/max(self.true_observation.variable_cardinalities[var_name])
        question_context = self.text_generator.statement_text_past(var_cat,var_name,num_val,self.true_observation)

        return question_statement,question_context
    
    def get_distractors(self,right_decision:HeuristicDecision,restricted_actions=[HeuristicDecisionMSG.ELICIT_TARGET,HeuristicDecisionMSG.ELICIT_GENERAL],new_person=True):
        possible_actions = []
        if restricted_actions is not None:
            action_set = restricted_actions
        else:
            action_set = HeuristicDecision.interesting_actions

        if new_person:
            bodies = self.bodies + ["NEWPERSON"]
        else:
            bodies = self.bodies
    
        for i in action_set:
            if HeuristicDecision.takes_target[i]:
                for body in bodies:
                    if not(i == right_decision.action and body == right_decision.target):
                        possible_actions.append((i,body))
            else:
                if i != right_decision.action:
                    possible_actions.append((i,None))
        return possible_actions
    
    def follow_up_new_person(self):
        if self.var_cats[self.variables[0]] in ["GENERAL","ROBOT"]:
            return "",[],[]

        # TODO: Change based on real people's distances
        new_person = {
            "Group":False,
            "Group Confidence":1,
            "Motion":MotionActivity.NOTHING,
            "Motion Confidence":1,
            "Engagement Level":EngagementLevel.DISENGAGED,
            "Engagement Level Confidence":1,
            "Group with Robot":False,
            "Mutual Gaze":1,
            "Engagement Value":0.5,
            "Pose Estimation Confidence":1,
            "Distance":2,
            "Waiting":False,
        }

        new_state = copy.deepcopy(self.true_observation.state)
        new_state["NEWPERSON"] = new_person

        # New observation
        observation = EngageStateObservation(new_state,self.bodies+["NEWPERSON"])
        var_name = self.var_names[self.variables[0]]
        intervention_order = self.counterfactual.intervention_order+["NEWPERSON_{}".format(var_name)]
        context_text = self.text_generator.question_context_imaginary_person_absolute(var_name,new_person,self.person_name)
        

        question_texts = []
        outcomes = []
        for val in observation.variable_cardinalities[var_name]:
            cont_val = observation.real_value(var_name,val)
            intervention = {"NEWPERSON":{var_name:cont_val}}
            outcome = self.counterfactual.outcome(observation,intervention_order,intervention)
            outcomes.append(outcome)
            q_text = self.text_generator.question_text_imaginary_person_absolute(var_name,cont_val,self.true_observation,self.person_name)
            question_texts.append(q_text)

        return context_text,question_texts,outcomes
    
    def generate_distractors(self,right_answer:HeuristicOutcome,restricted_actions=None):
        distractors = self.get_distractors(right_answer,restricted_actions=restricted_actions)
        chosen_distractor_indices = np.random.choice(range(len(distractors)), 2, replace=False)
        chosen_distractors = [HeuristicOutcome(HeuristicDecision(distractors[i][0],distractors[i][1])) for i in chosen_distractor_indices]

        answer_outcomes = [right_answer] + chosen_distractors
        indices = [0,1,2]
        np.random.shuffle(indices)
        right_answer_index = indices.index(0)

        answer_texts = []
        answer_components = {"Actions":[],"Targets":[]}
        for i in indices:
            answer_texts.append(self.future_outcome_text(answer_outcomes[i]))
            answer_components["Actions"].append(answer_outcomes[i].string_action())
            answer_components["Targets"].append(answer_outcomes[i].string_target())
        
        return answer_texts,right_answer_index,answer_components
