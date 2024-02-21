import rospy
import argparse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import copy

from sensor_msgs.msg import Image

from engage.decision_maker.decision_manager import DecisionManager
from engage.explanation.hri_explainer import HRIExplainer
from engage.explanation.heuristic_lime_explainer import HeuristicLimeExplainer
from explanation_msgs.msg import Explainability

class LiveExplainer:
    explainers = {
        "counterfactual":HRIExplainer,
        "heuristic_lime":HeuristicLimeExplainer,
    }

    def __init__(self,
                 rgb_image_topic,
                 decision_maker,
                 groups = [0,1,2],
                 buffer_time=5,
                 explainer="counterfactual",
                 rate=20) -> None:
        self.rate = rospy.Rate(rate)
        self.decision_maker = decision_maker
        self.buffer_length = rate*buffer_time

        # Decision-Maker
        # TODO: The parameters of the decision maker should match the real ones ... how?
        self.dm = DecisionManager.decision_makers[decision_maker]()

        # Explainer
        self.explainer_name = explainer
        self.explainer = self.explainers[explainer](self.dm)

        self.cv_bridge = CvBridge()

        # Subscribers
        rgb_img_sub = rospy.Subscriber(rgb_image_topic,Image,callback=self.update_image_buffer)
        dec_sub = rospy.Subscriber(
            "/hri_engage/decision_states",
            DecisionManager.decision_state_msgs[decision_maker],
            callback=self.decision_state_callback
            )
        
        # Publishers
        self.et_pub = rospy.Publisher("/explanation_test/explanation",Explainability,queue_size=1)
        
        # Image buffer
        self.image_buffer = []
        self.image_times = []

        # Pose buffer
        self.pose_buffer = []
        self.pose_times = []

        # Explainability Test
        self.var_nums = self.explainer.obs_type.variable_counter()
        self.groups = groups
        self.curr_group_index = 0

    def update_image_buffer(self,rgb_img):
        self.image_buffer.append(rgb_img)
        self.image_times.append(rgb_img.header.stamp)

        if len(self.image_buffer)>self.buffer_length:
            self.image_buffer = self.image_buffer[-self.buffer_length:]
            self.image_times = self.image_times[-self.buffer_length:]

    def decision_state_callback(self,dec):
        dec_time = dec.header.stamp

        if self.dm.decision.interesting_decision(dec.decision):
            # Only handle decisions that are interesting, not e.g. NOTHING or WAITING
            try:
                dec_index = self.image_times.index(dec_time)
            except ValueError:
                print("No image found at time: ",dec_time)
                return None
            
            dec_img = self.image_buffer[dec_index]

            # Set up explainer
            self.explainer.setup_explanation(dec,query=None,decision_maker=self.decision_maker)

            # Explain
            #self.explainer.explain()
            explainability_test = self.explainer.generate_explainability_test(self.groups[self.curr_group_index],self.var_nums)
            self.publish_explainability_test(explainability_test,dec_img)

            # Update group
            self.curr_group_index = (self.curr_group_index + 1) % len(self.groups)

    def save_image(self,img):
        try:
            cv2_img = self.cv_bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError:
            raise CvBridgeError
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('/home/tamlin/engage/latest_decision.jpeg', cv2_img)

    def publish_explainability_test(self,et_test,image):
        message = et_test.to_message(image)
        print(message)
        self.et_pub.publish(message)


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("HRILiveExplainer", anonymous=True)

    default_camera = "camera"
    parser = argparse.ArgumentParser(description="Explain interactions live")
    parser.add_argument("-i", "--rgb_image_topic", help="Topic for rgb image",
                        type=str, default="/{}/color/image_raw".format(default_camera))
    parser.add_argument("-d", "--decision_maker", help="Which decision maker will be used",
                        type=str, default="heuristic")
    parser.add_argument("--buffer_time", help="How long the node will store images for",
                        type=int, default=5)
    parser.add_argument("--explainer", help="Which explainer will be used",
                        type=str, default="counterfactual")
    parser.add_argument("--groups", help="Which groups to include. all = [0,1,2]. control = [0]. nocf = [1]. full = [2]. nomid = [0,2].",
                        type=str, default="all")
    args = parser.parse_args()

    if args.groups == "all":
        groups = [0,1,2]
    elif args.groups == "control":
        groups = [0]
    elif args.groups == "nocf":
        groups = [1]
    elif args.groups == "full":
        groups = [2]
    elif args.groups == "nomid":
        groups = [0,2]
    else:
        raise Exception(args.groups)

    explainer = LiveExplainer(
        args.rgb_image_topic,
        args.decision_maker,
        buffer_time=args.buffer_time,
        explainer=args.explainer,
        groups=groups,
    )
    explainer.run()