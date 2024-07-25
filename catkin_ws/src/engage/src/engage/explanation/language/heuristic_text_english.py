from engage.explanation.language.heuristic_text import HeuristicText

class HeuristicTextEnglish(HeuristicText):
    confidence_val_texts = {
        0.00:"very unsure",
        0.33:"unsure",
        0.67:"reasonably sure",
        1.00:"very sure",
    }

    def __init__(self) -> None:
        super().__init__()

    def outcome_text_conditional(self,outcome,person_name):
        if outcome.action == self.dmsg.NOTHING:
            text = "I would have done nothing"
        elif outcome.action == self.dmsg.WAIT:
            text = "I would have finished what I was doing"
        elif outcome.action == self.dmsg.MAINTAIN:
            text = "I would have tried to keep interacting with {}".format(person_name(outcome.target))
        elif outcome.action == self.dmsg.RECAPTURE:
            text = "I would have tried to recapture the attention of {}".format(person_name(outcome.target))
        elif outcome.action == self.dmsg.ELICIT_GENERAL:
            text = "I would have tried to attract attention from anybody around me"
        elif outcome.action == self.dmsg.ELICIT_TARGET:
            text = "I would have tried to get {} to talk with me".format(person_name(outcome.target))
        else:
            raise ValueError
        
        return text

    def outcome_text_past(self,outcome,person_name):
        if outcome.action == self.dmsg.NOTHING:
            text = "I did nothing."
        elif outcome.action == self.dmsg.WAIT:
            text = "I waited for my action to execute."
        elif outcome.action == self.dmsg.MAINTAIN:
            text = "I tried to keep interacting with {}.".format(person_name(outcome.target))
        elif outcome.action == self.dmsg.RECAPTURE:
            text = "I tried to recapture the attention of {}.".format(person_name(outcome.target))
        elif outcome.action == self.dmsg.ELICIT_GENERAL:
            text = "I tried to attract attention from anyone in general."
        elif outcome.action == self.dmsg.ELICIT_TARGET:
            text = "I tried to get {} to talk with me.".format(person_name(outcome.target))
        else:
            raise ValueError
        
        return text
    
    def outcome_text_future(self,outcome,person_name):
        if outcome.action == self.dmsg.NOTHING:
            text = "I will do nothing"
        elif outcome.action == self.dmsg.WAIT:
            text = "I will wait for my action to execute"
        elif outcome.action == self.dmsg.MAINTAIN:
            text = "I will try to keep interacting with {}".format(person_name(outcome.target))
        elif outcome.action == self.dmsg.RECAPTURE:
            text = "I will try to recapture the attention of {}".format(person_name(outcome.target))
        elif outcome.action == self.dmsg.ELICIT_GENERAL:
            text = "I will try to attract attention from anyone around me"
        elif outcome.action == self.dmsg.ELICIT_TARGET:
            text = "I will try to get {} to talk with me".format(person_name(outcome.target))
        else:
            raise ValueError
        
        return text
    
    def confidence_reason_text(self,var_name,subject,threshold,approx_val,thresh_min,approx_thresh):       
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


        return "I was {} about {}.".format(feeling,var_string) 
    
    def confidence_counterfactual_texts(self,var,var_name,var_cats,values,subject,min_val,max_val):
        if var_cats[var] == "ROBOT":
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
    
    def reason_text(self,var,var_name,var_cats,subject,explanation_values,true_val,true_observation):
        if true_observation.variable_categories[var_name] == "Categorical":
            # Categorical variables, pretty straightforward
            if var_name == "Group":
                if var_cats[var] == "ROBOT":
                    subject =  "I"
                
                if true_val:
                    negation = ""
                    end_text = "with someone"
                else:
                    negation = "not "
                    end_text = "with anyone"

                return "{} was {}in a group {}.".format(subject,negation,end_text)
            elif var_name == "Motion":
                if true_val == self.motion.NOTHING:
                    return "{} wasn't moving.".format(subject)
                elif true_val == self.motion.WALKING_AWAY:
                    return "{} was walking away from me.".format(subject)
                elif true_val == self.motion.WALKING_TOWARDS:
                    return "{} was walking towards me.".format(subject)
                elif true_val == self.motion.WALKING_PAST:
                    return "{} was walking past me.".format(subject)
            elif var_name == "Engagement Level":
                if true_val == self.eng_lvl.UNKNOWN:
                    return "I wasn't sure if {} was interested in me or not.".format(subject)
                elif true_val == self.eng_lvl.DISENGAGED:
                    return "It seemed {} was not interested in me.".format(subject)
                elif true_val == self.eng_lvl.ENGAGING:
                    return "It seemed {} was starting to get interested in me.".format(subject)
                elif true_val == self.eng_lvl.ENGAGED:
                    return "It seemed {} was interested in me.".format(subject)
                elif true_val == self.eng_lvl.DISENGAGING:
                    return "It seemed {} was starting to lose interest in me.".format(subject)
            elif var_name == "Group with Robot":
                if true_val:
                    return "{} was in a group with me.".format(subject)
                else:
                    return "{} was not in a group with me.".format(subject)
            elif var_name == "Waiting":
                if true_val:
                    return "I was doing something else."
                else:
                    return "I wasn't doing anything else."
        else:
            # Continuous variables...much trickier
            approx_val = round(true_val,2)
            min_range = min(explanation_values[var])
            max_range = max(explanation_values[var])

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
                return self.confidence_reason_text(var_name,subject,threshold,approx_val,thresh_min,approx_thresh)
            elif var_name == "Mutual Gaze":
                if threshold is None:
                    if approx_val == 0.00:
                        return "{} was not looking at me at all.".format(subject)
                    elif approx_val == 0.33:
                        return "{} was not really looking at me.".format(subject)
                    elif approx_val == 0.67:
                        return "{} and I were sort of looking at each other.".format(subject)
                    elif approx_val == 1.00:
                        return "{} and I were looking directly at each other.".format(subject)
                else:
                    if thresh_min:
                        # True value is lower than a threshold
                        if approx_thresh == 0.33:
                            return "{} and I were not looking at each other at all.".format(subject)
                        elif approx_thresh == 0.67:
                            return "{} and I were not looking at each other.".format(subject)
                        elif approx_thresh == 1.00:
                            return "{} and I were not looking directly at each other.".format(subject)
                    else:
                        # True value is higher than a threshold
                        if approx_thresh == 0.67:
                            return "{} and I were looking directly at each other.".format(subject)
                        elif approx_thresh == 0.33:
                            return "{} and I were looking at each other.".format(subject)
                        elif approx_thresh == 0.00:
                            return "{} wasn't looking away from me.".format(subject)
            elif var_name == "Engagement Value":
                if threshold is None:
                    if approx_val == 0.00:
                        return "{} was not engaged with me at all.".format(subject)
                    elif approx_val == 0.33:
                        return "{} was not engaged with me.".format(subject)
                    elif approx_val == 0.67:
                        return "{} was quite engaged with me.".format(subject)
                    elif approx_val == 1.00:
                        return "{} was very engaged with me.".format(subject)
                else:
                    if thresh_min:
                        # True value is lower than a threshold
                        if approx_thresh == 0.33:
                            return "{} was not at all engaged with me.".format(subject)
                        elif approx_thresh == 0.67:
                            return "{} was not particularly engaged with me.".format(subject)
                        elif approx_thresh == 1.00:
                            return "{} and was not very engaged with me.".format(subject)
                    else:
                        # True value is higher than a threshold
                        if approx_thresh == 0.67:
                            return "{} was very engaged with me.".format(subject)
                        elif approx_thresh == 0.33:
                            return "{} was at least a little engaged with me.".format(subject)
                        elif approx_thresh == 0.00:
                            return "{} wasn't completely unenengaged with me.".format(subject)
            elif var_name == "Distance":
                if threshold is None:
                    return "{} was about {}m from me.".format(subject,approx_val)
                else:
                    if thresh_min:
                        # True value is lower than a threshold
                        return "{} was closer than {}m from me.".format(subject,approx_thresh)
                    else:
                        # True value is higher than a threshold
                        return "{} was further than {}m from me.".format(subject,approx_thresh)
                    
    def construct_reason_text(self,true_outcome,var,var_name,var_cats,subject,explanation_values,true_val,true_observation,person_name):
        return self.outcome_text_past(true_outcome,person_name)[:-1] + " because " + self.reason_text(var,var_name,var_cats,subject,explanation_values,true_val,true_observation)
    
    def construct_counterfactual_text(self,foil_text,counterfactual_decision_text):
        return " If {}, {}.".format(foil_text,counterfactual_decision_text)
    
    def foil_text(self,var,var_name,var_cats,values,subject,true_values,true_observation,cards):
        val_settings = []
        if true_observation.variable_categories[var_name] == "Categorical":
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
                cards_removed.remove(true_values[var])

                if sorted(values) == sorted(cards_removed):
                    # Any change
                    val_settings.append("{} was doing anything else".format(subject))
                else:
                    # Specific changes
                    for val in values:
                        if val == self.motion.NOTHING:
                            val_settings.append("{} was doing nothing".format(subject))
                        elif val == self.motion.WALKING_AWAY:
                            val_settings.append("{} was walking away from me".format(subject))
                        elif val == self.motion.WALKING_TOWARDS:
                            val_settings.append("{} was walking towards me".format(subject))
                        elif val == self.motion.WALKING_PAST:
                            val_settings.append("{} was walking past me".format(subject))
            elif var_name == "Engagement Level":
                # Check if it consists of every possible change
                cards_removed = cards.copy()
                cards_removed.remove(true_values[var])

                if sorted(values) == sorted(cards_removed):
                    # Any change
                    val_settings.append("{}'s level of interest was different in any way".format(subject))
                else:
                    # Specific changes
                    for val in values:
                        if val == self.eng_lvl.UNKNOWN:
                            val_settings.append("I didn't know whether or not {} was interested in me".format(subject))
                        elif val == self.eng_lvl.DISENGAGED:
                            val_settings.append("{} was not interested in me".format(subject))
                        elif val == self.eng_lvl.ENGAGING:
                            val_settings.append("{} was starting to become interested in me".format(subject))
                        elif val == self.eng_lvl.ENGAGED:
                            val_settings.append("{} was interested in me".format(subject))
                        elif val == self.eng_lvl.DISENGAGING:
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
                val_settings.append(self.confidence_counterfactual_texts(var,var_name,var_cats,sorted_vals,subject,min_val,max_val))
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
        
    def statement_text_conditional(self,var_cat,var_name,value,subject,true_observation):
        return self.statement_text_past(var_cat,var_name,value,subject,true_observation)
        
    def statement_text_past(self,var_cat,var_name,value,subject,true_observation):
        if true_observation.variable_categories[var_name] == "Categorical":
            if var_name == "Group":
                if subject == "ROBOT":
                    subject = "I"

                if value:
                   return "{} was in a group with someone".format(subject)
                else:
                    return "{} was not in a group anyone".format(subject)
            elif var_name == "Motion":
                if value == self.motion.NOTHING:
                    return "{} was doing nothing".format(subject)
                elif value == self.motion.WALKING_AWAY:
                    return "{} was walking away from me".format(subject)
                elif value == self.motion.WALKING_TOWARDS:
                    return "{} was walking towards me".format(subject)
                elif value == self.motion.WALKING_PAST:
                    return "{} was walking past me".format(subject)
            elif var_name == "Engagement Level":
                if value == self.eng_lvl.UNKNOWN:
                    return "I didn't know whether or not {} was interested in me".format(subject)
                elif value == self.eng_lvl.DISENGAGED:
                    return "{} was not interested in me".format(subject)
                elif value == self.eng_lvl.ENGAGING:
                    return "{} was starting to become interested in me".format(subject)
                elif value == self.eng_lvl.ENGAGED:
                    return "{} was interested in me".format(subject)
                elif value == self.eng_lvl.DISENGAGING:
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
    
    def question_context(self,context_statement):
        return "When I made the decision, {}.".format(context_statement)
    
    def question_text(self,question_statement):
        return "What do you think I would do if {}?".format(question_statement)
    
    def question_context_imaginary_person_absolute(self,var_name,new_person):
        context_text = "Imagine there was a person, Bob, who was "

        if var_name != "Mutual Gaze":
            if new_person["Mutual Gaze"] == 0:
                mg_text = "not looking at me at all"
            elif new_person["Mutual Gaze"] == 0.33:
                mg_text = "not really looking at me"
            elif new_person["Mutual Gaze"] == 0.67:
                mg_text = "looking in my general direction"
            elif new_person["Mutual Gaze"] == 1:
                mg_text = "looking directly at me"

        if var_name == "Mutual Gaze":
            context_text += "standing {}m away from me.".format(new_person["Distance"])
        elif var_name == "Distance":
            context_text += "{}.".format(mg_text)
        else:
            context_text += "standing {}m away from me and {}.".format(new_person["Distance"],mg_text)
        return context_text
    
    def question_text_imaginary_person_absolute(self,var_name,value,true_observation,person_name):

        return "What do you think I would do if {}?".format(self.statement_text_conditional("NEWPERSON",var_name,value,person_name("NEWPERSON"),true_observation))
        
        