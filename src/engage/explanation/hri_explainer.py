import numpy as np
import argparse
import copy
import itertools
import collections
import networkx as nx

from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.msg import Decision
from engage.explanation.counterfactual_explainer import Counterfactual,CounterfactualExplainer,Observation,Outcome
from engage.explanation.hri_causal_model import SimpleCausalModel
from engage.decision_helper import DecisionState

class HRIBodyOutcome(Outcome):
    def __init__(self,action,target):
        self.target = target
        if isinstance(action,str):
            self.action = HeuristicDecision.action_names.index(action)
        else:
            self.action = action

        if self.action is not None:
            self.action_string = HeuristicDecision.action_names[self.action]
        else:
            self.action_string = None

    def __eq__(self, other):
        if type(other) is type(self):
            return self.__dict__ == other.__dict__
        return False
    
    def __str__(self):
        return "<{},{}>".format(self.action_string,self.target)
    
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
    
            
class HRIBodyObservation(Observation):
    variable_categories = {
        "Group":"Categorical",
        "Group Confidence":"Continuous",
        "Motion":"Categorical",
        "Motion Confidence":"Continuous",
        "Engagement Level":"Categorical",
        "Engagement Level Confidence":"Continuous",
        "Group with Robot":"Categorical",
        "Mutual Gaze":"Continuous",
        "Engagement Value":"Continuous",
        "Pose Estimation Confidence":"Continuous",
        "Distance":"Continuous",
        "Waiting":"Categorical",
    }

    variable_cardinalities = {
        "Group":[False,True],
        "Group Confidence":[0,1,2,3],
        "Motion":[0,1,2,3],
        "Motion Confidence":[0,1,2,3],
        "Engagement Level":[0,1,2,3,4],
        "Engagement Level Confidence":[0,1,2,3],
        "Group with Robot":[False,True],
        "Mutual Gaze":[0,1,2,3],
        "Engagement Value":[0,1,2,3],
        "Pose Estimation Confidence":[0,1,2,3],
        "Distance":[0.1,0.5,1,1.5,2,2.5,3,4,5,6,7,8,9],
        "Waiting":[False,True],
    }

    def __init__(self,state,bodies):
        self.state = state
        self.bodies = bodies

        self.influences = self.get_influences()
        self.causal_graph = SimpleCausalModel(self.bodies,HRIBodyObservation.variable_cardinalities)

    def get_influences(self):
        influences = []
        for key in self.state:
            if key == "DECISION":
                continue
            for variable in self.state[key]:
                influences.append("{}_{}".format(key,variable))
        return influences
    
    def critical_interventions(self,interventions,var):
        intervention_list = []
        var_name = var.split("_")
        key = var_name[0]
        var = var_name[1]
        clist = self.variable_cardinalities[var].copy()
        clist.remove(self.state[key][var])

        for changed_val in clist:
            new_intervention = copy.deepcopy(interventions)
            if key not in new_intervention:
                new_intervention[key] = {}
            new_intervention[key][var] = changed_val

            intervention_list.append(new_intervention)
        return intervention_list
    
    def critical_interventions_multi(self,interventions,vars):
        intervention_list = []
        clists = []
        for var in vars:
            var_name = var.split("_")
            key = var_name[0]
            var = var_name[1]
            clist = self.variable_cardinalities[var].copy()
            clist.remove(self.state[key][var])
            clists.append(clist)
        asses = list(itertools.product(*clists))
        
        for ass in asses:
            new_intervention = copy.deepcopy(interventions)
            for var,var_val in zip(vars,ass):

                var_name = var.split("_")
                key = var_name[0]
                var = var_name[1]
                if key not in new_intervention:
                    new_intervention[key] = {}
                new_intervention[key][var] = var_val
                
            intervention_list.append(new_intervention)
        return intervention_list
    
    def get_influence_type(self,var):
        var_name = var.split("_")
        var = var_name[1]

        return self.variable_categories[var]
    
    def value_of_variable_in_assignment(self,var,assignment,discrete=False):
        var_name = var.split("_")
        discrete_val = assignment[var_name[0]][var_name[1]]
        if discrete or self.variable_categories[var_name[1]] == "Categorical":
            return discrete_val
        elif self.variable_categories[var_name[1]] == "Continuous" and var_name[1]=="Distance":
            return discrete_val
        else:
            return discrete_val/max(self.variable_cardinalities[var_name[1]])
        
    def form_critical_set(self,variable,values):
        var_name = variable.split("_")
        var_range = self.variable_cardinalities[var_name[1]].copy()
        var_range.remove(self.state[var_name[0]][var_name[1]])

        if self.variable_categories[var_name[1]] == "Categorical":
            if set(values) == set(var_range):
                return True
            else:
                return False
        else:
            valid_vals = []
            for val in var_range:
                valid_vals.append(val in values)
            ta,tb = self.calculate_threshold_index(valid_vals)
            if ta is not None:
                return True
            else:
                return False


    
    def __str__(self) -> str:
        return str(self.state)
    
class HRIBodyCounterfactual(Counterfactual):
    def __init__(self,decision_maker,intervention_order=[],interventions={},changes={}):
        '''
        intervention_order = ["waiting","aaaaa_G",etc.]

        interventions:
            {
            "Waiting":...,
            "aaaaa":{
                "G":...,
                "GC"...,
                etc.
            },
            "bbbbb":{
                "G":...,
                "GC"...,
                etc.
            },
            etc.
            }
        '''
        self.decision_maker = decision_maker
        self.intervention_order = intervention_order
        self.interventions = interventions
        self.changes = changes

    def copy(self):
        return HRIBodyCounterfactual(self.decision_maker,intervention_order=self.intervention_order,interventions=copy.deepcopy(self.interventions),changes=copy.deepcopy(self.changes))
    
    def full_state(self,observation,changes):
        state = copy.deepcopy(observation.state)
        for key in changes:
            for variable in changes[key]:
                state[key][variable] = changes[key][variable]

        return state
    
    def outcome(self,observation,intervention_order=None,interventions=None):
        if intervention_order is None:
            intervention_order = self.intervention_order
        if interventions is None:
            interventions = self.interventions

        changes = self.apply_interventions(observation,intervention_order,interventions)

        state = self.full_state(observation,changes)
        
        decision_state = DecisionState(from_decision_node=False,state=state,bodies=observation.bodies)

        dm_outcome = self.decision_maker.decide(decision_state)

        return HRIBodyOutcome(dm_outcome.action,dm_outcome.target)
    
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
    
class HRIBodyExplainer:
    def __init__(self,decision_maker):
        self.decision_maker = decision_maker

    def setup_explanation(self,state,state_bodies,true_decision,query):
        self.true_observation = HRIBodyObservation(state,state_bodies)
        self.true_outcome = HRIBodyOutcome(true_decision[0],true_decision[1])
        self.query = HRIBodyOutcome(query[0],query[1])
        self.bodies = state_bodies

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
                elif self.query.action in [Decision.NOTHING,Decision.WAIT,Decision.ELICIT_GENERAL]:
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

        cfx = CounterfactualExplainer(self.true_observation,self.true_outcome,HRIBodyCounterfactual,self.decision_maker)
        critical_influences,critical_thresholds,critical_values = cfx.explain(self.query,max_depth)

        if len(critical_influences)>0:
            return critical_influences,critical_thresholds,critical_values,True, None
        else:
            return None,None,None,False,"No explanations found for max_depth: {}".format(max_depth)