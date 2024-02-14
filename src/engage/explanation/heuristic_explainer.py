import copy

from engage.explanation.counterfactual_explainer import Outcome,Query,Counterfactual,Explanation
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.engage_state import EngageState
from engage.explanation.engage_state_observation import EngageStateObservation
from engage.msg import HeuristicDecision as HeuristicDecisionMSG, MotionActivity, EngagementLevel
      
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
    confidence_val_texts = {
        0.00:"very unsure",
        0.33:"unsure",
        0.67:"reasonably sure",
        1.00:"very sure",
    }

    def __init__(self,true_outcome:HeuristicOutcome,true_observation:EngageStateObservation,query:HeuristicQuery,counterfactual:HeuristicCounterfactual):
        self.true_observation = true_observation
        self.true_outcome = true_outcome
        self.query = query
        self.counterfactual = counterfactual
        self.bodies = self.get_bodies()
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
        
    def present_explanation(self,counterfactual=True):
        explanation_text = self.true_action_text() + " because " + self.reason_text() + "."
        if counterfactual:
            explanation_text += self.counterfactual_explanation_component()
        return explanation_text

    def true_action_text(self):
        if self.true_outcome.action == HeuristicDecisionMSG.NOTHING:
            text = "I did nothing"
        elif self.true_outcome.action == HeuristicDecisionMSG.WAIT:
            text = "I waited for my action to execute"
        elif self.true_outcome.action == HeuristicDecisionMSG.MAINTAIN:
            text = "I tried to keep interacting with {}".format(self.person_name(self.true_outcome.target))
        elif self.true_outcome.action == HeuristicDecisionMSG.RECAPTURE:
            text = "I tried to recapture the attention of {}".format(self.person_name(self.true_outcome.target))
        elif self.true_outcome.action == HeuristicDecisionMSG.ELICIT_GENERAL:
            text = "I tried to attract attention from anyone around me"
        elif self.true_outcome.action == HeuristicDecisionMSG.ELICIT_TARGET:
            text = "I tried to get {} to talk with me".format(self.person_name(self.true_outcome.target))
        else:
            raise ValueError
        
        return text
    
    def reason_text(self):
        if self.num_vars == 1:
            return self.reason_text_var(self.variables[0],self.true_values[self.variables[0]])
        else:
            raise NotImplementedError("Still need to implement multi-variable explanations")
        
    def reason_text_var(self,var,true_val):
        var_name = self.var_names[var]
        subject =  self.person_name(self.var_cats[var])

        if self.true_observation.variable_categories[var_name] == "Categorical":
            # Categorical variables, pretty straightforward
            if var_name == "Group":
                if self.var_cats[var] == "ROBOT":
                    subject =  "I"
                
                if true_val:
                    negation = ""
                    end_text = "with someone"
                else:
                    negation = "not "
                    end_text = "with anyone"

                return "{} was {}in a group {}".format(subject,negation,end_text)
            elif var_name == "Motion":
                if true_val == MotionActivity.NOTHING:
                    return "{} wasn't moving".format(subject)
                elif true_val == MotionActivity.WALKING_AWAY:
                    return "{} was walking away from me".format(subject)
                elif true_val == MotionActivity.WALKING_TOWARDS:
                    return "{} was walking towards me".format(subject)
                elif true_val == MotionActivity.WALKING_PAST:
                    return "{} was walking past me".format(subject)
            elif var_name == "Engagement Level":
                if true_val == EngagementLevel.UNKNOWN:
                    return "I wasn't sure if {} was interested in me or not".format(subject)
                elif true_val == EngagementLevel.DISENGAGED:
                    return "It seemed {} was not interested in me".format(subject)
                elif true_val == EngagementLevel.ENGAGING:
                    return "It seemed {} was starting to get interested in me".format(subject)
                elif true_val == EngagementLevel.ENGAGED:
                    return "It seemed {} was interested in me".format(subject)
                elif true_val == EngagementLevel.DISENGAGING:
                    return "It seemed {} was starting to lose interest in me".format(subject)
            elif var_name == "Group with Robot":
                if true_val:
                    return "{} was in a group with me".format(subject)
                else:
                    return "{} was not in a group with me".format(subject)
            elif var_name == "Waiting":
                if true_val:
                    return "I was doing something else"
                else:
                    return "I wasn't doing anything else"
        else:
            # Continuous variables...much trickier
            approx_val = round(true_val,2)
            min_range = min(self.explanation_values[var])
            max_range = max(self.explanation_values[var])

            thresh_min = None
            if true_val < min_range:
                # All counterfactuals are higher
                threshold = min_range
                thresh_min = True
                approx_thresh = round(threshold,2)
            elif true_val > max_range:
                # All counterfactuals are lower:
                threshold = max_range
                thresh_min = False
                approx_thresh = round(threshold,2)
            else:
                # True value is somewhere in between, meaning a more complex answer
                threshold = None
                approx_thresh = None

            #
            # Can start comparing var_name
            #
            if var_name in ["Group Confidence","Motion Confidence","Engagement Level Confidence","Pose Estimation Confidence"]:
                return self.confidence_reason_text(var,var_name,threshold,approx_val,thresh_min,approx_thresh)
            elif var_name == "Mutual Gaze":
                if threshold is None:
                    if approx_val == 0.00:
                        return "{} was not looking at me at all".format(subject)
                    elif approx_val == 0.33:
                        return "{} was not really looking at me".format(subject)
                    elif approx_val == 0.67:
                        return "{} and I were sort of looking at each other".format(subject)
                    elif approx_val == 1.00:
                        return "{} and I were looking directly at each other".format(subject)
                else:
                    if thresh_min:
                        # True value is lower than a threshold
                        if approx_thresh == 0.33:
                            return "{} and I were not looking at each other at all".format(subject)
                        elif approx_thresh == 0.67:
                            return "{} and I were not looking at each other".format(subject)
                        elif approx_thresh == 1.00:
                            return "{} and I were not looking directly at each other".format(subject)
                    else:
                        # True value is higher than a threshold
                        if approx_thresh == 0.67:
                            return "{} and I were looking directly at each other".format(subject)
                        elif approx_thresh == 0.33:
                            return "{} and I were looking at each other".format(subject)
                        elif approx_thresh == 0.00:
                            return "{} wasn't looking away from me".format(subject)
            elif var_name == "Engagement Value":
                if threshold is None:
                    if approx_val == 0.00:
                        return "{} was not engaged with me at all".format(subject)
                    elif approx_val == 0.33:
                        return "{} was not engaged with me".format(subject)
                    elif approx_val == 0.67:
                        return "{} was quite engaged with me".format(subject)
                    elif approx_val == 1.00:
                        return "{} was very engaged with me".format(subject)
                else:
                    if thresh_min:
                        # True value is lower than a threshold
                        if approx_thresh == 0.33:
                            return "{} was not at all engaged with me".format(subject)
                        elif approx_thresh == 0.67:
                            return "{} was not particularly engaged with me".format(subject)
                        elif approx_thresh == 1.00:
                            return "{} and was not very engaged with me".format(subject)
                    else:
                        # True value is higher than a threshold
                        if approx_thresh == 0.67:
                            return "{} was very engaged with me".format(subject)
                        elif approx_thresh == 0.33:
                            return "{} was at least a little engaged with me".format(subject)
                        elif approx_thresh == 0.00:
                            return "{} wasn't completely unenengaged with me".format(subject)
            elif var_name == "Distance":
                if threshold is None:
                    return "{} was about {}m from me".format(subject,approx_val)
                else:
                    if thresh_min:
                        # True value is lower than a threshold
                        return "{} was closer than {}m from me".format(subject,approx_thresh)
                    else:
                        # True value is higher than a threshold
                        return "{} was further than {}m from me".format(subject,approx_thresh)
                  

    def confidence_reason_text(self,var,var_name,threshold,approx_val,thresh_min,approx_thresh):
        # Subject
        if self.var_cats[var] == "ROBOT":
            subject = "I"
        else:
            subject = self.person_name(self.var_cats[var])
        
        # Threshold
        if threshold is None:
            feeling = self.confidence_val_texts[approx_val]  
        else:
            if thresh_min:
                # True value is lower than a threshold
                if approx_thresh == 0.33:
                    feeling = "very unsure"
                else:
                    feeling = "not {}".format(self.confidence_val_texts[approx_thresh])
            else:
                # True value is greater than a threshold
                if approx_thresh == 0.67:
                    feeling = "very sure"
                else:
                    feeling = "not {}".format(self.confidence_val_texts[approx_thresh])

        # Variable type
        if var_name == "Group Confidence":
            var_string = "whether or not {} was in a group".format(subject)
        elif var_name == "Motion Confidence":
            var_string = "{}'s motion".format(subject)
        elif var_name == "Engagement Level Confidence":
            var_string = "whether or not {} was interested in me".format(subject)
        elif var_name == "Pose Estimation Confidence":
            var_string = "my detection of {}'s skeleton".format(subject)


        return "I was {} about {}".format(feeling,var_string) 
    
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
            counterfactual_decision_text = self.counterfactual_decision_text(outcome_objects[outcome])
            counterfactual_components.append(" If {}, I would have {}.".format(foil_text,counterfactual_decision_text))

        # Concantenate
        counterfactual_text = ""
        for comp in counterfactual_components:
            counterfactual_text += comp
        return counterfactual_text


    def foil_text_single_var(self,var,values):
        var_name = self.var_names[var]
        subject =  self.person_name(self.var_cats[var])
        cards = self.true_observation.variable_cardinalities[var_name]

        val_settings = []
        if self.true_observation.variable_categories[var_name] == "Categorical":
            
            if var_name == "Group":
                if subject == "ROBOT":
                    subject = "I"

                if values[0]:
                    negation = ""
                else:
                    negation = "not "

                val_settings.append("{} was {}in a group".format(subject,negation))
            elif var_name == "Motion":
                # Check if it consists of every possible change
                cards_removed = cards.copy()
                cards_removed.remove(self.true_values[var])

                if sorted(values) == sorted(cards_removed):
                    # Any change
                    val_settings.append("{} was doing anything else".format(subject))
                else:
                    # Specific changes
                    for val in values:
                        if val == MotionActivity.NOTHING:
                            val_settings.append("{} was doing nothing".format(subject))
                        elif val == MotionActivity.WALKING_AWAY:
                            val_settings.append("{} was walking away from me".format(subject))
                        elif val == MotionActivity.WALKING_TOWARDS:
                            val_settings.append("{} was walking towards me".format(subject))
                        elif val == MotionActivity.WALKING_PAST:
                            val_settings.append("{} was walking past me".format(subject))
            elif var_name == "Engagement Level":
                # Check if it consists of every possible change
                cards_removed = cards.copy()
                cards_removed.remove(self.true_values[var])

                if sorted(values) == sorted(cards_removed):
                    # Any change
                    val_settings.append("{}'s level of interest was different in any way".format(subject))
                else:
                    # Specific changes
                    for val in values:
                        if val == EngagementLevel.UNKNOWN:
                            val_settings.append("I didn't know whether or not {} was interested in me".format(subject))
                        elif val == EngagementLevel.DISENGAGED:
                            val_settings.append("{} was not interested in me".format(subject))
                        elif val == EngagementLevel.ENGAGING:
                            val_settings.append("{} was starting to become interested in me".format(subject))
                        elif val == EngagementLevel.ENGAGED:
                            val_settings.append("{} was interested in me".format(subject))
                        elif val == EngagementLevel.DISENGAGING:
                            val_settings.append("{} was starting to lose interest in me".format(subject))
            elif var_name == "Group with Robot":
                if values[0]:
                    val_settings.append("{} was in a group with me".format(subject))
                else:
                    val_settings.append("{} was not in a group with me".format(subject))
            elif var_name == "Waiting":
                if values[0]:
                    val_settings.append("I was doing something else")
                else:
                    val_settings.append("I was not busy doing something")

        else:
            if var_name != "Distance":
                min_range = 0.00
                max_range = 1.00
            else:
                max_range = round(max(cards),2)
                min_range = round(min(cards),2)

            sorted_vals = sorted(values)
            approx_max = round(sorted_vals[-1],2)
            approx_min = round(sorted_vals[0],2)

            min_val = approx_min == min_range
            max_val = approx_max == max_range
            
            if var_name in ["Group Confidence","Motion Confidence","Engagement Level Confidence","Pose Estimation Confidence"]:
                val_settings.append(self.confidence_counterfactual_texts(var,var_name,sorted_vals,subject,min_val,max_val))
            elif var_name == "Mutual Gaze":
                if len(sorted_vals) == 1:
                    approx_val = round(sorted_vals[0],2)

                    if approx_val == 0.00:
                        val_settings.append("{} was not looking at me at all".format(subject))
                    elif approx_val == 0.33:
                        val_settings.append("{} was not really looking at me".format(subject))
                    elif approx_val == 0.67:
                        val_settings.append("{} and I were sort of looking at each other".format(subject))
                    elif approx_val == 1.00:
                        val_settings.append("{} and I were looking at each other".format(subject))

                else:
                    if min_val:
                        # Only concerns the highest value
                        

                        if approx_max == 0.33:
                            val_settings.append("{} was not looking at me".format(subject))
                        elif approx_max == 0.67:
                            val_settings.append("{} and I were not directly looking at each other".format(subject))
                        elif approx_max == 1.00:
                            # Somewhere in between
                            val_settings.append("{} and I were either looking directly at each other or directly away from eachother".format(subject))

                    elif max_val:
                        # Only conerns the lowest value
                        

                        if approx_min == 0.67:
                            val_settings.append("{} was looking at me".format(subject))
                        elif approx_min == 0.33:
                            val_settings.append("{} was not looking directly away from me".format(subject))
                        elif approx_min == 0.00:
                            # Somewhere in between
                            val_settings.append("{} and I were either looking directly at each other or directly away from eachother".format(subject))
                    else:
                        if approx_min == 0.33 and approx_max == 0.67:
                            # This is the only valid combination unless another discretisation is introduced
                            val_settings.append("{} was neither looking directly at me nor directly away from me".format(subject))
            elif var_name == "Engagement Value":
                if len(sorted_vals) == 1:
                    approx_val = round(sorted_vals[0],2)

                    if approx_val == 0.00:
                        val_settings.append("{} was not engaged with me at all".format(subject))
                    elif approx_val == 0.33:
                        val_settings.append("{} was not really engaged with me".format(subject))
                    elif approx_val == 0.67:
                        val_settings.append("{} was pretty engaged with me".format(subject))
                    elif approx_val == 1.00:
                        val_settings.append("{} was very engaged with me".format(subject))

                else:
                    if min_val:
                        # Only concerns the highest value

                        if approx_max == 0.33:
                            val_settings.append("{} was not so engaged with me".format(subject))
                        elif approx_max == 0.67:
                            val_settings.append("{} was not very engaged with me".format(subject))
                        elif approx_max == 1.00:
                            # Somewhere in between
                            val_settings.append("{} was either very engaged with me or not very engaged with me".format(subject))

                    elif max_val:
                        # Only conerns the lowest value

                        if approx_min == 0.67:
                            val_settings.append("{} was engaged with me".format(subject))
                        elif approx_min == 0.33:
                            val_settings.append("{} was at least a little engaged with me".format(subject))
                    else:
                        if approx_min == 0.33 and approx_max == 0.67:
                            # This is the only valid combination unless another discretisation is introduced
                            val_settings.append("{} was neither very disengaged nor very engaged with me".format(subject))
            elif var_name == "Distance":
                if len(sorted_vals) == 1:
                    approx_val = round(sorted_vals[0],2)
                    val_settings.append("{} was about {}m away from me".format(subject,approx_val))
                else:
                    if min_val:
                        val_settings.append("{} was {}m away or closer".format(subject,approx_max))
                    elif max_val:
                        val_settings.append("{} was {}m away or further".format(subject,approx_min))
                    else:
                        val_settings.append("{} was between {}m and {}m away".format(subject,approx_min,approx_max))

            

        if len(val_settings) == 0:
            error_message = "No setting for {}:{} (min: {}, max: {})".format(var,values,min_val,max_val)
            raise ValueError(error_message)
        elif len(val_settings) == 1:
            return val_settings[0]
        else:
            foil_text = ""
            for val_set in val_settings:
                foil_text += val_set + ", or "
            return foil_text[:-5]
    def confidence_counterfactual_texts(self,var,var_name,values,subject,min_val,max_val):
        if self.var_cats[var] == "ROBOT":
            subject = "I"

        if min_val:
            # Only concerns the highest value
            approx_max = round(values[-1],2)

            if approx_max == 0.00:
                situation = "I was very unsure"
            elif approx_max == 0.33:
                situation = "I wasn't reasonably sure"
            elif approx_max == 0.67:
                situation = "I wasn't very sure"
            else:
                # Somewhere in between
                situation = "I was either very sure or very unsure"

        elif max_val:
            # Only conerns the lowest value
            approx_min = round(values[0],2)

            if approx_min == 1.00:
                situation = "I was very sure"
            elif approx_min == 0.67:
                situation = "I was reasonably sure"
            elif approx_min == 0.33:
                situation = "I was at least a little bit sure"
            else:
                # Somewhere in between
                situation = "I was either very sure or very unsure"
        else:
            # Concerns both
            if len(values) == 1:
                situation = "I was {}".format(self.confidence_val_texts[round(values[0],2)])
            else:
                approx_min = round(values[0],2)
                approx_max = round(values[-1],2)
                if approx_min == 0.33 and approx_max == 0.67:
                    # This is the only valid combination unless another discretisation is introduced
                    situation = "I was a little sure, but not too sure,"
                    
        
        if var_name == "Group Confidence":
            var_string = "whether or not {} was in a group".format(subject)
        elif var_name == "Motion Confidence":
            var_string = "{}'s motion".format(subject)
        elif var_name == "Engagement Level Confidence":
            var_string = "whether or not {} was interested in me".format(subject)
        elif var_name == "Pose Estimation Confidence":
            var_string = "my detection of {}'s skeleton".format(subject)

        return "{} about {}".format(situation,var_string)



    def counterfactual_decision_text(self,outcome):
        if outcome.action == HeuristicDecisionMSG.NOTHING:
            text = "done nothing"
        elif outcome.action == HeuristicDecisionMSG.WAIT:
            text = "finished what I was doing"
        elif outcome.action == HeuristicDecisionMSG.MAINTAIN:
            text = "tried to keep interacting with {}".format(self.person_name(outcome.target))
        elif outcome.action == HeuristicDecisionMSG.RECAPTURE:
            text = "tried to recapture the attention of {}".format(self.person_name(outcome.target))
        elif outcome.action == HeuristicDecisionMSG.ELICIT_GENERAL:
            text = "tried to attract attention from anybody around me"
        elif outcome.action == HeuristicDecisionMSG.ELICIT_TARGET:
            text = "tried to get {} to talk with me".format(self.person_name(outcome.target))
        else:
            raise ValueError
        
        return text

    def person_name(self,target_name):
        # TODO: Map targets to better, more readable names
        return target_name
    
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
                fuq_text,fuq_prelude = self.question_text(fuq[0][0],fuq[0][1],fuq[1])
                fuq_text = "When I made the decision, {}. What do you think I would do if {}?".format(fuq_prelude,fuq_text)
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
        question = self.statement_text(var_cat,var_name,value)

        num_val = self.true_observation.state[var_cat][var_name]
        if self.true_observation.variable_categories[var_name] == "Continuous" and var_name!="Distance":
            num_val = self.true_observation.state[var_cat][var_name]/max(self.true_observation.variable_cardinalities[var_name])
        prelude = self.statement_text(var_cat,var_name,num_val)

        return question,prelude
    
    def statement_text(self,var_cat,var_name,value):
        subject = self.person_name(var_cat)
        if self.true_observation.variable_categories[var_name] == "Categorical":
            if var_name == "Group":
                if subject == "ROBOT":
                    subject = "I"

                if value:
                   return "{} was in a group with someone".format(subject)
                else:
                    return "{} was not in a group anyone".format(subject)
            elif var_name == "Motion":
                if value == MotionActivity.NOTHING:
                    return "{} was doing nothing".format(subject)
                elif value == MotionActivity.WALKING_AWAY:
                    return "{} was walking away from me".format(subject)
                elif value == MotionActivity.WALKING_TOWARDS:
                    return "{} was walking towards me".format(subject)
                elif value == MotionActivity.WALKING_PAST:
                    return "{} was walking past me".format(subject)
            elif var_name == "Engagement Level":
                if value == EngagementLevel.UNKNOWN:
                    return "I didn't know whether or not {} was interested in me".format(subject)
                elif value == EngagementLevel.DISENGAGED:
                    return "{} was not interested in me".format(subject)
                elif value == EngagementLevel.ENGAGING:
                    return "{} was starting to become interested in me".format(subject)
                elif value == EngagementLevel.ENGAGED:
                    return "{} was interested in me".format(subject)
                elif value == EngagementLevel.DISENGAGING:
                    return "{} was starting to lose interest in me".format(subject)
            elif var_name == "Group with Robot":
                if value:
                    return "{} was in a group with me".format(subject)
                else:
                    return "{} was not in a group with me".format(subject)
            elif var_name == "Waiting":
                if value:
                    return "I was doing something else"
                else:
                    return "I was not doing something else"
        else:
            if var_name in ["Group Confidence","Motion Confidence","Engagement Level Confidence","Pose Estimation Confidence"]:
                # Subject
                if subject == "ROBOT":
                    subject = "I"
                
                # Threshold
                feeling = self.confidence_val_texts[round(value,2)]

                # Variable type
                if var_name == "Group Confidence":
                    var_string = "whether or not {} was in a group".format(subject)
                elif var_name == "Motion Confidence":
                    var_string = "{}'s motion".format(subject)
                elif var_name == "Engagement Level Confidence":
                    var_string = "whether or not {} was interested in me".format(subject)
                elif var_name == "Pose Estimation Confidence":
                    var_string = "my detection of {}'s skeleton".format(subject)


                return "I was {} about {}".format(feeling,var_string)
            elif var_name == "Mutual Gaze":
                approx_val = round(value,2)
                if approx_val == 0.00:
                    return "{} was looking directly away from me".format(subject)
                elif approx_val == 0.33:
                    return "{} was not really looking at me".format(subject)
                elif approx_val == 0.67:
                    return "{} was looking mostly in my direction".format(subject)
                elif approx_val == 1.00:
                    return "{} was looking directly at me".format(subject)
            elif var_name == "Engagement Value":
                approx_val = round(value,2)
                if approx_val == 0.00:
                    return "{} was not engaged with me at all".format(subject)
                elif approx_val == 0.33:
                    return "{} was not particularly engaged with me".format(subject)
                elif approx_val == 0.67:
                    return "{} was somewhat engaged with me".format(subject)
                elif approx_val == 1.00:
                    return "{} was very engaged with me".format(subject)
            elif var_name == "Distance":
                approx_val = round(value,2)
                return "{} was {}m away from me".format(subject,approx_val)
            
        # Shouldn't get here
        raise Exception(var_cat,var_name,value)
    
    def get_distractors(self,right_decision:HeuristicDecision,restricted_actions=[HeuristicDecisionMSG.ELICIT_TARGET,HeuristicDecisionMSG.ELICIT_GENERAL]):
        possible_actions = []
        if restricted_actions is not None:
            action_set = restricted_actions
        else:
            action_set = HeuristicDecision.interesting_actions
        for i in action_set:
            if HeuristicDecision.takes_target[i]:
                for body in self.bodies:
                    if not(i == right_decision.action and body == right_decision.target):
                        possible_actions.append((i,body))
            else:
                if i != right_decision.action:
                    possible_actions.append((i,None))
        return possible_actions
