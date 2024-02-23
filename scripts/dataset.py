import rospy
import argparse
import numpy as np
import string
import copy
import pandas as pd
import random

from engage.explanation.engage_state_observation import EngageStateObservation
from engage.decision_maker.engage_state import EngageState
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.simple_target_decision_maker import SimpleTargetDecisionMaker

class DatasetGenerator:
    def __init__(self,
                 num_bodies=2,
                 N = 10000,
                 rate=20,
                 **kwargs):
        self.rate = rospy.Rate(rate)
        self.N = N

        self.decision_maker = SimpleTargetDecisionMaker()
        self.state,self.bodies = self.create_state(num_bodies)

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
                if HeuristicDecision.action_names[action] in ["MAINTAIN","RECAPTURE"]:
                    continue
                for body in self.bodies:
                    self.action_indices[action][body] = i
                    self.vector_actions.append((action,body))
                    self.action_names.append("{}_{}".format(HeuristicDecision.action_names[action],body))
                    i += 1
            else:
                if HeuristicDecision.action_names[action] == "NOTHING":
                    continue
                self.action_indices[action] = i
                self.vector_actions.append((action,None))
                self.action_names.append("{}".format(HeuristicDecision.action_names[action]))
                i += 1
        self.num_actions = len(self.vector_actions)

        dataset = self.fabricate_data_new(self.N)
        actions = self.predict(dataset)

        action_name_map = {}
        for i in range(self.num_actions):
            action_name_map[self.action_names[i]] = i
        print(action_name_map)

        df = pd.DataFrame.from_records(dataset,columns=self.var_names)
        df["Decision"] = actions
        
        # Prune
        for body in self.bodies:
            vars_to_drop = ["Motion","Motion Confidence","Group","Group Confidence","Engagement Level","Group with Robot","Engagement Level Confidence"]
            df = df.drop(["{}_{}".format(body,var) for var in vars_to_drop],axis="columns")
        df.to_csv('simple_hri_dataset_{}.csv'.format(self.N))
        print("Saved!")
        
    def create_state(self,num_bodies):
        bodies = ["Person {}".format(string.ascii_uppercase[i]) for i in range(num_bodies)]
        #state = {"GENERAL":{"Waiting":False},"ROBOT":{"Group":False,"Group Confidence":0}}
        state = {"GENERAL":{"Waiting":False}}
        for body in bodies:
            state[body] = {}
            for var in ["Distance","Mutual Gaze","Engagement Value","Pose Estimation Confidence","Motion","Motion Confidence","Group","Group Confidence","Engagement Level","Group with Robot","Engagement Level Confidence"]:
            #for var in ["Distance","Mutual Gaze","Engagement Value","Pose Estimation Confidence"]:
                state[body][var] = 0
        return state,bodies
    
    def fabricate_data_new(self,N):
        data = []
        for i in range(N):
            state = copy.deepcopy(self.state)
            state["GENERAL"]["Waiting"] = np.random.choice([True,False])
            for body in self.bodies:
                state[body]["Distance"] = random.uniform(0,8)
                state[body]["Mutual Gaze"] = random.uniform(0,1)
                state[body]["Pose Estimation Confidence"] = random.uniform(0,1)
                state[body]["Engagement Value"] = min(state[body]["Mutual Gaze"]/state[body]["Distance"],1)
            state_vec = self.state_to_vec(state)
            data.append(state_vec)
        return np.array(data)


    def fabricate_data(self,N):
        data = []
        state_observation = EngageStateObservation(self.state,self.bodies)
        for i in range(N):
            sample_state = state_observation.sample_state()
            state_vec = self.state_to_vec(sample_state)
            data.append(state_vec)
        return np.array(data)
    
    def predict(self,data):
        decisions = []
        for i in range(data.shape[0]):
            state = self.vec_to_state(data[i,:])
            decision_state = EngageState()
            decision_state.dict_to_state(state,self.bodies)

            decision = self.decision_maker.decide_simple(decision_state)
            dec = self.decision_number(decision)
            decisions.append(dec)
        return np.array(decisions)
    
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
    
    def decision_number(self,decision):
        if HeuristicDecision.takes_target[decision.action]:
            return self.action_indices[decision.action][decision.target]
        else:
            return self.action_indices[decision.action]
    
    def decision_to_vec(self,decision):
        action_vector = np.zeros(self.num_actions)
        if HeuristicDecision.takes_target[decision.action]:
            action_vector[self.action_indices[decision.action][decision.target]] = 1
        else:
            action_vector[self.action_indices[decision.action]] = 1

        return action_vector
    
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("HRIDatasetGenerator", anonymous=True)

    parser = argparse.ArgumentParser(description="Manage the predictions experiment")
    parser.add_argument("-n","--sample_size", help="How many observations",
                        type=int, default=10000)
    args = parser.parse_args(rospy.myargv()[1:])

    explainer = DatasetGenerator(
        N=args.sample_size,
    )
    explainer.run()