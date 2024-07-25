import rospy
import argparse
import rosbag
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
import time

from explanation_msgs.msg import Explainability

class ScreenshotMaker:
    def __init__(
            self,
            image,
            rate=20,
    ) -> None:
        self.rate = rospy.Rate(rate)
        self.bridge = CvBridge()

        # Bag
        
        topic = "/input_explanation"

        # Load Image
        img = cv2.imread(image)
        ros_img = self.bridge.cv2_to_compressed_imgmsg(img)

        # Explanations
        explanations = [
            "I tried to talk to Person A.",
            "I tried to talk to Person A because Person C was not very interested in me.",
            "I tried to talk to Person A because Person C was not very interested in me.",
        ]
        counterfactuals = [
            "",
            "",
            "If Person C was very interested in me, I would have tried to talk to Person C."
        ]
        for g in [0,1,2]:
            bag = rosbag.Bag('paper_{}.bag'.format(g), 'w')
            msg = self.make_message(ros_img,g,explanations[g],counterfactuals[g])
            bag.write(topic,msg)
            bag.close()
        print("Done")
        exit()

    def make_message(self,ros_img,group,explanation,counterfactual):
        msg = Explainability()
        msg.header.stamp = rospy.Time.now()
        msg.group = group
        msg.played_before = False
        msg.image = ros_img
        msg.blank_image = ros_img
        msg.explanation = explanation
        msg.counterfactual = counterfactual
        msg.question_context = "Imagine that there is an additional person, Bob, who is 2m away from me and looking at me directly."
        msg.question_text = "What do you think I would do if Bob was not interested in me at all?"
        msg.answer_list = ["I would try to get anyone to talk to me","I would try to talk to Person A","I would try to talk to Bob"]
        msg.correct_answer = 2
        msg.user_answer = 0
        msg.true_decision_action = "ELICIT_TARGET"
        msg.true_decision_target = "abcde"
        msg.explanation_variable = "Engagement Value"
        msg.explanation_target = "cdefg"
        msg.answer_actions = ["ELICIT_GENERAL","ELICIT_TARGET","ELICIT_TARGET"]
        msg.answer_targets = ["None","abcde","NEWPERSON"]
        return msg


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--image", help="Image", default=os.getenv("HOME")+"/engage/image.jpeg")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("ScreenshotMaker", anonymous=True)
    
    sm = ScreenshotMaker(
        image = args.image,
        )
    sm.run()