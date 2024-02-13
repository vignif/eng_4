import itertools
import copy

from engage.explanation.explanation_helper import CounterfactualTree

class Query:
    def __init__(self):
        pass

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
        self.decision_maker = decision_maker
        self.intervention_order = intervention_order
        self.interventions = interventions
        self.changes = changes

    def copy(self):
        raise NotImplementedError

    def outcome(self,real_data):
        '''
        Return the decision maker's outcome for this counterfactual
        '''
        raise NotImplementedError
    
    def full_state(self,observation,changes):
        state = copy.deepcopy(observation.state)
        for key in changes:
            for variable in changes[key]:
                state[key][variable] = changes[key][variable]

        return state
    
    def apply_interventions(self,observation,intervention_order,interventions,causal=True):
        changes = {}
        if causal:
            causal_graph = copy.deepcopy(observation.causal_graph)
            
            for intrv in intervention_order:
                if intrv in causal_graph.graph.nodes:
                    # In the order of interventions, remove the edges from parents of intervened node
                    parents = list(causal_graph.graph.predecessors(intrv))
                    for par in parents:
                        causal_graph.graph.remove_edge(par,intrv)
                    # Apply causal effects
                    changes = self.apply_change(changes,interventions,observation,intrv)
                    changes = self.apply_causal_effects(causal_graph,changes,observation,intrv)
                else:
                    # Just apply regularly
                    changes = self.apply_change(changes,interventions,observation,intrv)
        else:
            for intrv in intervention_order:
                changes = self.apply_change(changes,interventions,observation,intrv)
        return changes
    
    def apply_causal_effects(self,causal_graph,changes,observation,intrv):
        children = causal_graph.graph.successors(intrv)
        for child in children:
            changes = causal_graph.get_causal_effect(intrv,child,observation.state,changes)
        # Recursively apply down the graph
        children = causal_graph.graph.successors(intrv)
        for child in children:
            changes = self.apply_causal_effects(causal_graph,changes,observation,child)

        return changes

    def apply_change(self,changes,interventions,observation,var):

        intrv_list = var.split("_")
        if intrv_list[0] not in changes:
                changes[intrv_list[0]] = {}
        changes[intrv_list[0]][intrv_list[1]] = interventions[intrv_list[0]][intrv_list[1]]

        return changes
    
    def in_interventions(self,influence_var):
        if influence_var in self.interventions:
            return True
        
        vlist = influence_var.split("_")
        if len(vlist)==2 and vlist[0] in self.interventions and vlist[1] in self.interventions[vlist[0]]:
            return True
        
        return False
    
class Explanation:
    def __init__(self):
        self.num_vars = 0

    def make_critical_explanation(self,variable,values,outcome):
        raise NotImplementedError
    
class CounterfactualExplainer:
    def __init__(self,true_observation,true_outcome,counterfactual,decision_maker,explanation):
        self.true_observation = true_observation
        self.true_outcome = true_outcome
        self.CF = counterfactual
        self.decision_maker = decision_maker
        self.Explanation = explanation

    def explain(self,why_not,max_depth):
        influences = self.true_observation.get_influences()
        #print("{} influences".format(len(influences)))

        explanations = self.explain_case(influences,why_not,max_depth=max_depth)
        return explanations

    def explain_case(self,influences,why_not,counterfactual=None,max_depth=2):
        if counterfactual is None:
            counterfactual = self.CF(self.decision_maker)
        critical_influences,critical_thresholds,critical_values,outcomes = self.find_critical_influences(influences,why_not,counterfactual)
        explanations = self.assemble_critical_explanations(critical_influences,critical_values,outcomes,why_not,counterfactual)

        if explanations == []:
            # Need to find a partial explanation
            # TODO: proper loop here, maybe rename as not really potential, get the function to return something useful
            print("No critical influences found, will try deeper searches...")
            for i in range(2,max_depth+1):
                print("Explanations with {} variables".format(i))
                potential_influences = self.find_potential_influences(influences,why_not,counterfactual,depth=i)
                for exp in potential_influences:
                    print(exp)
        else:
            return explanations
        
    '''
    EXPLANATIONS
    '''
    def assemble_critical_explanations(self,critical_influences,critical_values,outcomes,query,counterfactual):
        explanations = []
        for i in range(len(critical_influences)):
            exp = self.Explanation(self.true_outcome,self.true_observation,query,counterfactual)
            exp.make_critical_explanation(critical_influences[i],critical_values[i],outcomes[i])
            explanations.append(exp)

        return explanations
    
    '''
    
    CRITICAL INFLUENCES
    
    '''
    def find_critical_influences(self,influences,why_not,counterfactual):
        critical_influences=[]
        thresholds = []
        critical_values = []
        counterfactual_outcomes = []

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
                    var_outcomes = []
                    for ci in critical_interventions:
                        outcome = counterfactual.outcome(self.true_observation,counterfactual.intervention_order+[var],ci)
                        
                        valid_outcome = self.true_outcome.valid_outcome(outcome,why_not)
                        var_outcomes.append(outcome)

                        if not valid_outcome:
                            all_valid = False
                            break

                    

                    if all_valid:
                        critical_influences.append(var)
                        thresholds.append(None)
                        critical_values.append([self.true_observation.value_of_variable_in_assignment(var,ci)])
                        counterfactual_outcomes.append(var_outcomes)
                elif vartype == "Continuous":
                    outcomes = []
                    values = []
                    var_outcomes = []
                    for ci in critical_interventions:
                        outcome = counterfactual.outcome(self.true_observation,counterfactual.intervention_order+[var],ci)
                        valid_outcome = self.true_outcome.valid_outcome(outcome,why_not)
                        var_outcomes.append(outcome)
                        outcomes.append(valid_outcome)
                        values.append(self.true_observation.value_of_variable_in_assignment(var,ci))
                    cia,cib = Observation.calculate_threshold_index(outcomes)
                    if cia is not None:
                        critical_influences.append(var)
                        thresholds.append((cia,cib))
                        critical_values.append([values[i] for i in range(cia,cib)])
                        counterfactual_outcomes.append([var_outcomes[i] for i in range(cia,cib)])

        return critical_influences,thresholds,critical_values,counterfactual_outcomes
    
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

        
            


            


            


            
                    
                    


            


                





        
