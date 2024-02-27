import rospy
import argparse
import os
import rosbag
import pandas as pd
from pathlib import Path

from explanation_msgs.msg import Explainability

class ResultsProcesser:
    def __init__(self,exp,bag_dir,rate=20) -> None:
        # ROS
        self.rate = rospy.Rate(rate)

        result_dir = bag_dir + "/prediction_experiment/"

        self.data_cols,self.data = self.setup_data()

        # Process each file
        self.read_files(exp,result_dir)

        # Combine into pandas df
        self.df = pd.DataFrame(self.data,columns=self.data_cols)
        print(self.df)

        # Save
        save_path = "{}/results/".format(bag_dir)
        Path(save_path).mkdir(parents=True, exist_ok=True)
        self.df.to_csv("{}{}.csv".format(save_path,exp))

        print("Saved to {}.csv".format(exp))


    '''
    
    SETUP

    '''
    def setup_data(self):
        data_cols = [
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
        ]

        return data_cols,[]

        

    '''
    READ
    '''
    def read_files(self,exp,result_dir):
        # Read each file
        for file in os.listdir(result_dir):
            filename = os.fsdecode(file)
            if filename.endswith(".bag") and filename.startswith(exp): 
                self.process_rosbag(result_dir,filename)
            else:
                continue

    def process_rosbag(self,result_dir,filename):
        file_path = os.path.join(result_dir, filename)
        print("Reading from {}".format(filename))
        bag = rosbag.Bag(file_path)
        for _,msg,_ in bag.read_messages(topics=["/out_explanation"]):
            row = [
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
            ]
            self.data.append(row)

        for _,msg,_ in bag.read_messages(topics=["/hri_engage/decision_states"]):
            print(msg)


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--exp", help="Name of experiment",
                        default="test")
    parser.add_argument("--bag_dir", help="Directory of bag file", default=os.getenv("HOME")+"/engage/src/engage/rosbags")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("ResultsProcesser", anonymous=True)
    
    proc = ResultsProcesser(
        args.exp,
        args.bag_dir,
        )
    proc.run()