import itertools
import copy
import networkx as nx

from engage.explanation.counterfactual_explainer import Observation
from engage.msg import EngagementLevel
from engage.decision_maker.engage_state import EngageState

class SimpleCausalModel:
    def __init__(self,bodies,cardinalities) -> None:
        self.bodies = bodies
        self.graph = self.get_causal_graph()
        self.cardinalities = cardinalities

    def get_causal_graph(self):
        G = nx.DiGraph()
        for body in self.bodies:
            if body != "ROBOT":
                edges = [
                    ("{}_Mutual Gaze".format(body),"{}_Engagement Value".format(body)),
                    ("{}_Distance".format(body),"{}_Engagement Value".format(body)),
                    ("{}_Engagement Value".format(body),"{}_Engagement Level".format(body)),
                    ("{}_Group with Robot".format(body),"ROBOT_Group"),
                    # TODO: Unsure if these should be included
                    ("{}_Engagement Value".format(body),"{}_Group with Robot".format(body)),
                    ("{}_Group with Robot".format(body),"{}_Group".format(body)),
                ]
                G.add_edges_from(edges)
        return G
    
    def get_causal_effect(self,u,v,state,changes):
        u_list = u.split("_")
        v_list = v.split("_")
        if u_list[0] in self.bodies and v_list[0] == u_list[0]:
            vars = ["Distance","Mutual Gaze","Engagement Value","Engagement Level"]
            var_vals = {k:state[u_list[0]][k] for k in vars}
            
            # Override with any changes
            if u_list[0] in changes:
                for var in vars:
                    if var in changes[u_list[0]]:
                        var_vals[var] = changes[u_list[0]][var]
            
            # Get causal effect
            if u_list[1] in ["Distance","Mutual Gaze"] and v_list[1] == "Engagement Value":
                new_ev = min(1,(var_vals["Mutual Gaze"]/max(self.cardinalities["Mutual Gaze"]))/var_vals["Distance"] if var_vals["Distance"] != 0 else 0)
                changes[u_list[0]]["Engagement Value"] = EngageState.float_bucket(new_ev)
            elif u_list[1] == "Engagement Value" and v_list[1] == "Engagement Level":
                # TODO: Implement something smarter here for estimating engagement level
                ev = var_vals["Engagement Value"]/max(self.cardinalities["Engagement Value"])
                if ev > 0.75:
                    el = EngagementLevel.ENGAGED
                elif ev > 0.5:
                    el = EngagementLevel.ENGAGING
                elif ev > 0.25:
                    el = EngagementLevel.DISENGAGING
                else:
                    el = EngagementLevel.DISENGAGED
                changes[u_list[0]]["Engagement Level"] = el
            elif u_list[1] == "Engagement Value" and v_list[1] == "Group with Robot":
                ev = var_vals["Engagement Value"]/max(self.cardinalities["Engagement Value"])
                if ev > 0.75:
                    changes[u_list[0]]["Group with Robot"] = True
                else:
                    changes[u_list[0]]["Group with Robot"] = False
            elif u_list[1] == "Group with Robot" and v_list[1] == "Group":
                gwr = state[u_list[0]]["Group with Robot"]
                if gwr:
                    changes[u_list[0]]["Group"] = True
                # TODO: logic for seeing if they are not in a group with anyone else and setting False if thats the case
                
        elif u_list[0] in self.bodies and u_list[1] == "Group with Robot" and v == "ROBOT_Group":
            var_vals = {body:state[body]["Group with Robot"] for body in self.bodies if body != "ROBOT"}
            # Override with any changes
            if u_list[0] in changes and "Group with Robot" in changes[u_list[0]]:
                var_vals[u_list[0]] = changes[u_list[0]]["Group with Robot"]
            # Get causal effect
            vals = list(var_vals.values())
            if not any(vals):
                if "ROBOT" not in changes:
                    changes["ROBOT"] = {"Group":False}
                else:
                    changes["ROBOT"]["Group"] = False
            else:
                if "ROBOT" not in changes:
                    changes["ROBOT"] = {"Group":True}
                else:
                    changes["ROBOT"]["Group"] = True
        return changes

class EngageStateObservation(Observation):
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
        self.causal_graph = SimpleCausalModel(self.bodies,EngageStateObservation.variable_cardinalities)

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