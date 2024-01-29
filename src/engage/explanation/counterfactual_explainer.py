import itertools

from engage.explanation.explanation_helper import CounterfactualTree

class Outcome:
    def __init__(self):
        pass

class Observation:
    def __init__(self):
        pass

    @staticmethod
    def calculate_threshold_index(valid_outcomes):
        # If there is one contiguous block of Trues, this will return the ends of that block. Otherwise None
        grouped_L = [(k, sum(1 for i in g)) for k,g in itertools.groupby(valid_outcomes)]
        if len(grouped_L)<=3 and grouped_L != []:
            if len(grouped_L)==3:
                # Only accept False...True...False
                if grouped_L[0][0] == grouped_L[2][0] and not grouped_L[0][0]:
                    return grouped_L[0][1],grouped_L[0][1]+grouped_L[1][1]
            elif len(grouped_L)==2:
                # Either False...True or True...False
                if grouped_L[0][0]:
                    return 0,grouped_L[0][1]
                else:
                    return grouped_L[0][1],len(valid_outcomes)
            else:
                # Only True or False
                if grouped_L[0][0]:
                    return 0,len(valid_outcomes)
        return None,None

class Counterfactual:
    def __init__(self,decision_maker,intervention_order=[],interventions=[],changes={}):
        pass

    def copy(self):
        raise NotImplementedError

    def outcome(self,real_data):
        '''
        Return the decision maker's outcome for this counterfactual
        '''
        raise NotImplementedError
    
class Explanation:
    def __init__(self,counterfactual,critical_influence=None):
        self.counterfactual = counterfactual
        self.critical_influence = critical_influence
    
class CounterfactualExplainer:
    def __init__(self,true_observation,true_outcome,counterfactual,decision_maker):
        self.true_observation = true_observation
        self.true_outcome = true_outcome
        self.CF = counterfactual
        self.decision_maker = decision_maker

    def explain(self,why_not,max_depth):
        influences = self.true_observation.get_influences()
        print("{} influences".format(len(influences)))

        critical_influences,critical_thresholds,critical_values = self.explain_case(influences,why_not,max_depth=max_depth)
        return critical_influences,critical_thresholds,critical_values

    def explain_case(self,influences,why_not,counterfactual=None,max_depth=2):
        if counterfactual is None:
            counterfactual = self.CF(self.decision_maker)
        critical_influences,critical_thresholds,critical_values = self.find_critical_influences(influences,why_not,counterfactual)

        if len(critical_influences)==0:
            # Need to find a partial explanation
            # TODO: proper loop here, maybe rename as not really potential, get the function to return something useful
            print("No critical influences found, will try deeper searches...")
            for i in range(2,max_depth+1):
                print("Explanations with {} variables".format(i))
                potential_influences = self.find_potential_influences(influences,why_not,counterfactual,depth=i)
            
        # TODO: Return an object for explanations
        else:
            return critical_influences,critical_thresholds,critical_values
    
    '''
    
    CRITICAL INFLUENCES
    
    '''
    def find_critical_influences(self,influences,why_not,counterfactual):
        critical_influences=[]
        thresholds = []
        critical_values = []

        for var in influences:
            if var in counterfactual.changes:
                # Ignore variables we have already changed
                continue
            
            critical_interventions = self.true_observation.critical_interventions(counterfactual.interventions,var)

            # TODO: Prune impossible states - maybe by checking causal assumptions, or even by applying DO or similar
            # e.g. for each assignment, get the changes

            if len(critical_interventions)>=1:
                all_valid = True
                vartype = self.true_observation.get_influence_type(var)

                if vartype == "Categorical":
                    for ci in critical_interventions:
                        outcome = counterfactual.outcome(self.true_observation,counterfactual.intervention_order+[var],ci)
                        valid_outcome = self.true_outcome.valid_outcome(outcome,why_not)

                        if not valid_outcome:
                            all_valid = False
                            break

                    

                    if all_valid:
                        critical_influences.append(var)
                        thresholds.append(None)
                        critical_values.append([self.true_observation.value_of_variable_in_assignment(var,ci)])
                elif vartype == "Continuous":
                    outcomes = []
                    values = []
                    for ci in critical_interventions:
                        outcome = counterfactual.outcome(self.true_observation,counterfactual.intervention_order+[var],ci)
                        valid_outcome = self.true_outcome.valid_outcome(outcome,why_not)

                        outcomes.append(valid_outcome)
                        values.append(self.true_observation.value_of_variable_in_assignment(var,ci))
                    cia,cib = Observation.calculate_threshold_index(outcomes)
                    if cia is not None:
                        critical_influences.append(var)
                        thresholds.append((cia,cib))
                        critical_values.append([values[i] for i in range(cia,cib)])

        return critical_influences,thresholds,critical_values
    
    '''
    
    POTENTIAL INFLUENCES
    
    '''

    def find_potential_influences(self,influences,why_not,counterfactual,depth):
        combos = itertools.combinations(influences, depth)
        valid_combos = []
        valid_assignments = []

        # Search each combination of size depth and see if assignments of interventions of this group of variables results in a valid outcome
        for combo in combos:
            critical_interventions = self.true_observation.critical_interventions_multi(counterfactual.interventions,combo)
            outcomes = []
            for ci in critical_interventions:
                outcome = counterfactual.outcome(self.true_observation,counterfactual.intervention_order+list(combo),ci)
                
                valid_outcome = self.true_outcome.valid_outcome(outcome,why_not)
                outcomes.append(valid_outcome)
                if valid_outcome:
                    valid_combos.append(combo)
                    valid_assignments.append(ci)

        # TODO: Attempt to find at least one critical influence in the groups of variables given their combos
        
        # Start by grouping counterfactuals of the same variable combinations in their interventions
        same_combo = {combo:[] for combo in valid_combos}
        
        for i in range(len(valid_combos)):
            same_combo[valid_combos[i]].append(valid_assignments[i])
        
        exps = []
        for combo in same_combo:
            tree = CounterfactualTree(combo,same_combo[combo],self.true_observation)
            exp = tree.merge_critical()
            exps.append(exp)

        return exps

        
            


            


            


            
                    
                    


            


                





        
