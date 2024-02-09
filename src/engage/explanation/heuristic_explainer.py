import copy

from engage.explanation.counterfactual_explainer import Outcome,Query,Counterfactual
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.engage_state import EngageState
      
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

        changes = self.apply_interventions(observation,intervention_order,interventions)

        state = self.full_state(observation,changes)
        
        decision_state = EngageState()
        decision_state.dict_to_state(state,observation.bodies)

        dm_outcome = self.decision_maker.decide(decision_state)

        return HeuristicOutcome(dm_outcome)
        

class HeuristicQuery(Query):
    def __init__(self,action=None,target=None):
        self.action = action
        self.target = target

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