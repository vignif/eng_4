import networkx as nx
from engage.msg import EngagementLevel

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
                changes[u_list[0]]["Engagement Value"] = DecisionState.float_bucket(new_ev)
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