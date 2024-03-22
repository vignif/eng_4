import rospy
import argparse
import os
import rosbag
import pandas as pd
from pathlib import Path
import re
import string

from cv_bridge import CvBridge, CvBridgeError
import cv2
import matplotlib.pyplot as plt


from explanation_msgs.msg import Explainability
from sensor_msgs.msg import CompressedImage

class ResultsProcesser:
    def __init__(self,exps,bag_dir,exp_name="PredictionExperiment",rate=20) -> None:
        # ROS
        self.rate = rospy.Rate(rate)

        result_dir = bag_dir + "/prediction_experiment/"

        self.data_cols,self.data = self.setup_data()

        # Debug
        self.bridge = CvBridge()

        # Process each file

        self.data = self.read_files(exps[0],result_dir)

        # Combine into pandas df
        self.df = pd.DataFrame(self.data,columns=self.data_cols)
        print(self.df)

        # Save
        save_path = "{}/results/".format(bag_dir)
        Path(save_path).mkdir(parents=True, exist_ok=True)
        self.df.to_csv("{}{}.csv".format(save_path,exp_name))

        print("Saved to {}.csv".format(exp_name))

        # Analyse
        groups = [0,1,2]
        for g in groups:
            df_g_t = self.df.loc[(self.df['Group'] == g) & (self.df["User Correct"] == True)]
            df_g_f = self.df.loc[(self.df['Group'] == g) & (self.df["User Correct"] == False)]
            print("For group {}, {} correct and {} incorrect".format(g,len(df_g_t),len(df_g_f)))

        for g in groups:
            df_g_t = self.df.loc[(self.df['Group'] == g) & (self.df["User Correct"] == True) & (self.df["Played Before"] == False)]
            df_g_f = self.df.loc[(self.df['Group'] == g) & (self.df["User Correct"] == False) & (self.df["Played Before"] == False)]
            print("Only including first timers, group {}, {} correct and {} incorrect".format(g,len(df_g_t),len(df_g_f)))

        for g in groups:
            df_g_t = self.df.loc[(self.df['Group'] == g) & (self.df["User Correct"] == True) & (self.df["Played Before"] == False) & (self.df["User Answer"] != 4)]
            df_g_f = self.df.loc[(self.df['Group'] == g) & (self.df["User Correct"] == False) & (self.df["Played Before"] == False) & (self.df["User Answer"] != 4)]
            print("Excluding don't know, group {}, {} correct and {} incorrect".format(g,len(df_g_t),len(df_g_f)))

        for g in groups:
            df_ft = self.df.loc[(self.df['Group'] == g) & (self.df["Played Before"] == False)]
            df_idk = self.df.loc[(self.df['Group'] == g) & (self.df["Played Before"] == False) & (self.df["User Answer"] == 4)]
            print("Number of I don't know for group {}: {} ({}%)".format(g,len(df_idk),100*len(df_idk)/len(df_ft)))

        # Variables
        vars = ["Distance","Mutual Gaze","Engagement Value","Pose Estimation Confidence"]
        for var in vars:
            df_v = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df["Played Before"] == False)]
            df_v_t = df_v.loc[df_v["User Correct"] == True]
            df_v_f = df_v.loc[df_v["User Correct"] == False]

            print("For var {}, overall {} correct ({}%) and {} incorrect ({}%)".format(var,
                                                                                       len(df_v_t),
                                                                                       100*len(df_v_t)/len(df_v),
                                                                                       len(df_v_f),
                                                                                       100*len(df_v_f)/len(df_v)))
        print("\n")
        
        num_var = {}
        num_g = {}
        num_var_ft = {}
        num_g_ft = {}
        num = 0
        num_ft = 0

        for var in vars:
            num_var[var] = 0
            num_var_ft[var] = 0
            for g in groups:
                if g not in num_g:
                    num_g[g] = 0
                    num_g_ft[g] = 0
                df_1 = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df['Group'] == g)]
                df_2 = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df["Played Before"] == False) & (self.df['Group'] == g)]
                print("For var {}, group {}, number = {} ({} ft)".format(var,g,len(df_1),len(df_2)))

                num_var[var] += len(df_1)
                num_g[g] += len(df_1)
                num_var_ft[var] += len(df_2)
                num_g_ft[g] += len(df_2)
                num += len(df_1)
                num_ft += len(df_2)

        print("Var nums: {}".format(num_var))
        print("Group nums: {}".format(num_g))
        print("Var nums 1st: {}".format(num_var_ft))
        print("Group nums 1st: {}".format(num_g_ft))
        print("Num: {}".format(num))
        print("Num 1st: {}".format(num_ft))


        for var in vars:
            for g in groups:
                df_v = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df["Played Before"] == False) & (self.df['Group'] == g)]
                df_v_t = df_v.loc[df_v["User Correct"] == True]
                df_v_f = df_v.loc[df_v["User Correct"] == False]

                print("For var {} and group {}, {} correct ({}%) and {} incorrect ({}%)".format(var,
                                                                                        g,
                                                                                        len(df_v_t),
                                                                                        100*len(df_v_t)/len(df_v),
                                                                                        len(df_v_f),
                                                                                        100*len(df_v_f)/len(df_v)))
            print("\n")

        for var in vars:
            df_v = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df["Played Before"] == False)]
            df_v_t = df_v.loc[df_v["User Correct"] == True]
            df_v_f = df_v.loc[df_v["User Correct"] == False]

            print("For var {}, overall {} correct ({}%) and {} incorrect ({}%)".format(var,
                                                                                       len(df_v_t),
                                                                                       100*len(df_v_t)/len(df_v),
                                                                                       len(df_v_f),
                                                                                       100*len(df_v_f)/len(df_v)))
        print("\nThe same, but without 'I don't know'\n")
        for var in vars:
            df_v = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df["Played Before"] == False) & (self.df["User Answer"] != 4)]
            df_v_t = df_v.loc[df_v["User Correct"] == True]
            df_v_f = df_v.loc[df_v["User Correct"] == False]

            print("For var {}, overall {} correct ({}%) and {} incorrect ({}%)".format(var,
                                                                                       len(df_v_t),
                                                                                       100*len(df_v_t)/len(df_v),
                                                                                       len(df_v_f),
                                                                                       100*len(df_v_f)/len(df_v)))
        print("\n")
            
        for var in vars:
            for g in groups:
                df_v = self.df.loc[(self.df["Explanation Variable"] == var) & (self.df["Played Before"] == False) & (self.df['Group'] == g) & (self.df["User Answer"] != 4)]
                df_v_t = df_v.loc[df_v["User Correct"] == True]
                df_v_f = df_v.loc[df_v["User Correct"] == False]

                print("For var {} and group {}, {} correct ({}%) and {} incorrect ({}%)".format(var,
                                                                                        g,
                                                                                        len(df_v_t),
                                                                                        100*len(df_v_t)/len(df_v),
                                                                                        len(df_v_f),
                                                                                        100*len(df_v_f)/len(df_v)))
            print("\n")

        df_t_n = self.df.loc[(self.df["User Correct"] == True) & (self.df["Played Before"] == False)]
        df_f_n = self.df.loc[(self.df["User Correct"] == False) & (self.df["Played Before"] == False)]
        df_t_o = self.df.loc[(self.df["User Correct"] == True) & (self.df["Played Before"] == True)]
        df_f_o = self.df.loc[(self.df["User Correct"] == False) & (self.df["Played Before"] == True)]

        print("First time success rate: {} ({}/{})".format(len(df_t_n)/(len(df_t_n)+len(df_f_n)),len(df_t_n),len(df_f_n)))
        print("Not first time success rate: {} ({}/{})".format(len(df_t_o)/(len(df_t_o)+len(df_f_o)),len(df_t_o),len(df_f_o)))



    '''
    
    SETUP

    '''
    def setup_data(self):
        data_cols = [
            "Time",
            "Group",
            "Played Before",
            "Explanation Reason",
            "Explanation Counterfactual",
            "Question Context",
            "Question Text",
            "Answer 1",
            "Answer 2",
            "Answer 3",
            "True Action",
            "True Target",
            "Explanation Variable",
            "Explanation Target",
            "Answer Action 1",
            "Answer Action 2",
            "Answer Action 3",
            "Answer Target 1",
            "Answer Target 2",
            "Answer Target 3",
            "Correct Answer",
            "User Answer",
            "User Correct",
            "Adjusted Answer",
            "Number of People",
            "Bag",
        ]

        return data_cols,[]

        

    '''
    READ
    '''
    def read_files(self,exps,result_dir):
        # Read each file
        data = []
        for file in os.listdir(result_dir):
            filename = os.fsdecode(file)
            for exp in exps:
                if filename.endswith(".bag") and filename.startswith(exp): # TODO: This would break if exp_i is a prefix of exp_j
                    data += self.process_rosbag(result_dir,filename)
                else:
                    continue
        return data
    

    def process_rosbag(self,result_dir,filename):
        file_path = os.path.join(result_dir, filename)
        print("Reading from {}".format(filename))
        bag = rosbag.Bag(file_path)
        data = []
        for _,msg,_ in bag.read_messages(topics=["/out_explanation"]):
            # Adjusted Answer
            adjusted_answer = 0
            if msg.correct_answer == msg.user_answer:
                adjusted_answer = 1
            elif msg.user_answer != 4 and msg.answer_actions[msg.user_answer-1] == "ELICIT_GENERAL":
                adjusted_answer = 0.5

            # Number of people
            text_num = self.highest_person_number(msg)
            relevant_decision = self.get_decision(msg.header.stamp,bag)
            if relevant_decision is None:
                num_people = 0
            else:
                num_people = len(relevant_decision.state.bodies)
            num_people = max(num_people,text_num)
            

            row = [
                msg.header.stamp,
                msg.group,
                msg.played_before,
                msg.explanation,
                msg.counterfactual,
                msg.question_context,
                msg.question_text,
                msg.answer_list[0],
                msg.answer_list[1],
                msg.answer_list[2],
                msg.true_decision_action,
                msg.true_decision_target,
                msg.explanation_variable,
                msg.explanation_target,
                msg.answer_actions[0],
                msg.answer_actions[1],
                msg.answer_actions[2],
                msg.answer_targets[0],
                msg.answer_targets[1],
                msg.answer_targets[2],
                msg.correct_answer,
                msg.user_answer,
                msg.correct_answer == msg.user_answer,
                adjusted_answer,
                num_people,
                filename,
            ]
            data.append(row)
        print("{} answers found in {}".format(len(data),filename))
        return data
    

    def get_decision(self,time,bag):
        times = []
        msgs = []
        for _,msg,_ in bag.read_messages(topics=["/hri_engage/decision_states"]):
            if msg.header.stamp == time:
                return msg
            times.append(msg.header.stamp)
            msgs.append(msg)
        return None
    
    def highest_person_number(self,msg):
        # Highest alphabetical number
        p_num_text = 0
        alphabets = ""
        for text in [msg.explanation,msg.counterfactual]:
            exp_splits = text.split(" ")
            for i in range(len(exp_splits)):
                if exp_splits[i] == "Persona":
                    alphabets += (re.sub('\W+',"",exp_splits[i+1]))
        if alphabets != "":
            p_num_text = string.ascii_uppercase.index(sorted(alphabets)[-1])+1
        
        answer_people = msg.answer_targets + [msg.true_decision_target]
        apeople = [person for person in answer_people if person not in ["None","NEWPERSON"]]
        apeople = list(set(apeople))

        return max(p_num_text,len(apeople))
        
        

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--exp", help="Name of experiment(s)",
                        action="append", nargs="+")
    parser.add_argument("--bag_dir", help="Directory of bag file", default=os.getenv("HOME")+"/engage/src/engage/rosbags")
    parser.add_argument("--name", help="Name for csv", default="PredictionExperiment")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("ResultsProcesser", anonymous=True)
    
    proc = ResultsProcesser(
        args.exp,
        args.bag_dir,
        exp_name=args.name,
        )
    proc.run()