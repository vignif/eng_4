import copy
import numpy as np

from engage.decision_maker.engage_state import EngageState
from engage.explanation.engage_state_observation import EngageStateObservation
from engage.decision_maker.decision_manager import DecisionManager
from engage.decision_maker.heuristic_decision import HeuristicDecision

import lime.lime_tabular

class HeuristicLimeExplainer:
    def __init__(self,decision_maker) -> None:
        self.decision_maker = decision_maker

    def setup_explanation(self,decision_state_msg,query=None,decision_maker="heuristic"):
        self.state,self.bodies = EngageState.single_state_from_msg(decision_state_msg)
        self.discrete_state = EngageState.discretise(self.state,self.bodies)
        self.true_observation = EngageStateObservation(self.discrete_state,self.bodies)
        self.true_outcome = DecisionManager.decision_outcomes[decision_maker](decision_state_msg.decision)

        # Vector stuff
        self.indices = copy.deepcopy(self.state)
        self.vector_vars = []
        self.var_names = []
        self.categorical_features = []
        i = 0
        for key in self.state:
            for var in self.state[key]:
                self.indices[key][var] = i
                self.vector_vars.append((key,var))
                self.var_names.append("{}_{}".format(key,var))
                if EngageStateObservation.variable_categories[var] == "Categorical":
                    self.categorical_features.append(i)
                i += 1
        self.num_vars = len(self.var_names)

        i = 0
        self.action_indices = {}
        self.vector_actions = []
        self.action_names = []
        
        for action in HeuristicDecision.takes_target:
            self.action_indices[action] = {}
            if HeuristicDecision.takes_target[action]:
                for body in self.bodies:
                    self.action_indices[action][body] = i
                    self.vector_actions.append((action,body))
                    self.action_names.append("{}_{}".format(HeuristicDecision.action_names[action],body))
                    i += 1
            else:
                self.action_indices[action] = i
                self.vector_actions.append((action,None))
                self.action_names.append("{}".format(HeuristicDecision.action_names[action]))
                i += 1
        self.num_actions = len(self.vector_actions)

        self.true_state_vec = self.state_to_vec(self.state)
        self.true_decision_vec = self.decision_to_vec(self.true_outcome)
        print("Creating sample data...")
        self.training_set = self.fabricate_training_set()
        print("Sample data created")
        self.explainer = lime.lime_tabular.LimeTabularExplainer(self.training_set, feature_names=self.var_names, class_names=self.action_names, discretize_continuous=True, categorical_features=self.categorical_features)

    def explain(self,**kwargs):
        print("===Observation===")
        print(self.true_observation)
        print("===Decision===")
        print(self.true_outcome)
        print("===LIME Explanation===")
        exp = self.explainer.explain_instance(self.true_state_vec, self.predict, num_features=10, top_labels=1)
        exp_mapping = exp.as_map()
        var_map = {}
        for act_i in exp_mapping:
            var_map[self.action_names[act_i]] = {}
            print(self.action_names[act_i])
            for i_prob in exp_mapping[act_i]:
                var_map[self.action_names[act_i]][self.var_names[i_prob[0]]] = i_prob[1]
                print("\t - {}:{}".format(self.var_names[i_prob[0]],i_prob[1]))
        
        #print(var_map)

    def fabricate_training_set(self,n=5000):
        data = []
        state_observation = EngageStateObservation(self.state,self.bodies)
        for i in range(n):
            sample_state = state_observation.sample_state()
            state_vec = self.state_to_vec(sample_state)
            data.append(state_vec)
        return np.array(data)

    def predict(self,data):
        probs = []
        for i in range(data.shape[0]):
            state = self.vec_to_state(data[i,:])
            decision_state = EngageState()
            decision_state.dict_to_state(state,self.bodies)

            decision = self.decision_maker.decide(decision_state)
            dec_vec = self.decision_to_vec(decision)
            probs.append(dec_vec)
        return np.array(probs)
        

    def state_to_vec(self,state):
        vec = []
        for state_var in self.vector_vars:
            vec.append(state[state_var[0]][state_var[1]])
        return np.array(vec)
    
    def vec_to_state(self,vector):
        state = copy.deepcopy(self.state)
        for i in range(len(vector)):
            var = self.vector_vars[i]
            # TODO: Undiscretise
            state[var[0]][var[1]] = vector[i]
        return state
    
    def decision_to_vec(self,decision):
        action_vector = np.zeros(self.num_actions)
        if HeuristicDecision.takes_target[decision.action]:
            action_vector[self.action_indices[decision.action][decision.target]] = 1
        else:
            action_vector[self.action_indices[decision.action]] = 1

        return action_vector