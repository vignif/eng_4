import rospy
import argparse
from cv_bridge import CvBridge, CvBridgeError
import cv2
import string
import numpy as np

from sensor_msgs.msg import Image

from engage.decision_maker.decision_manager import DecisionManager
from engage.explanation.hri_explainer import HRIExplainer
from engage.explanation.heuristic_lime_explainer import HeuristicLimeExplainer
from explanation_msgs.msg import Explainability
from engage.msg import PeoplePositions

class LiveExplainer:
    explainers = {
        "counterfactual":HRIExplainer,
        "heuristic_lime":HeuristicLimeExplainer,
    }

    def __init__(self,
                 rgb_image_topic,
                 pose_image_topic,
                 decision_maker,
                 groups = [0,1,2],
                 buffer_time=5,
                 explainer="counterfactual",
                 language="english",
                 use_pose=False,
                 rate=20) -> None:
        self.rate = rospy.Rate(rate)
        self.decision_maker = decision_maker
        self.buffer_length = rate*buffer_time
        self.language = language

        # Decision-Maker
        # TODO: The parameters of the decision maker should match the real ones ... how?
        self.dm = DecisionManager.decision_makers[decision_maker]()

        # Explainer
        self.explainer_name = explainer
        self.explainer = self.explainers[explainer](self.dm)

        self.cv_bridge = CvBridge()

        # Subscribers
        if use_pose:
            image_topic = pose_image_topic
        else:
            image_topic = rgb_image_topic

        img_sub = rospy.Subscriber(image_topic,Image,callback=self.update_image_buffer)
        position_sub = rospy.Subscriber("/humans/bodies/positions",PeoplePositions,callback=self.update_position_buffer)
        dec_sub = rospy.Subscriber(
            "/hri_engage/decision_states",
            DecisionManager.decision_state_msgs[decision_maker],
            callback=self.decision_state_callback
            )
        results_sub = rospy.Subscriber("/out_explanation",Explainability,callback=self.update_test_conditions)
        
        # Publishers
        self.et_pub = rospy.Publisher("/input_explanation",Explainability,queue_size=10)
        
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

    def update_position_buffer(self,positions:PeoplePositions):
        self.pose_buffer.append(positions)
        self.pose_times.append(positions.header.stamp)

        if len(self.pose_buffer)>self.buffer_length:
            self.pose_buffer = self.pose_buffer[-self.buffer_length:]
            self.pose_times = self.pose_times[-self.buffer_length:]

    def decision_state_callback(self,dec):
        dec_time = dec.header.stamp

        if self.dm.decision.interesting_decision(dec.decision):
            # Only handle decisions that are interesting, not e.g. NOTHING or WAITING
            dec_img,positions = self.get_decision_context(dec_time)
            # Get people name mapping
            names,ids = self.get_name_mapping(positions)

            # Process image, add labels
            img = self.ros_img_to_cv2(dec_img)
            img = self.draw_labels(img,positions,ids)
            #self.save_image(img)

            # Set up explainer
            self.explainer.setup_explanation(dec,query=None,decision_maker=self.decision_maker,names=names)

            # Explain
            explainability_test = self.explainer.generate_explainability_test(self.groups[self.curr_group_index],self.var_nums,language=self.language)
            if not explainability_test.no_explanations:
                print("Explanation found")
                self.publish_explainability_test(explainability_test,img,dec_time)
                #self.save_explainability_test(explainability_test)

               
            else:
                print("No explanations found")
                print(dec)

    def update_test_conditions(self,result_msg):
        # Update group
        self.curr_group_index = (self.curr_group_index + 1) % len(self.groups)

        # Update var nums
        self.var_nums[result_msg.explanation_variable] += 1

    def get_decision_context(self,time):
        try:
            dec_index = self.image_times.index(time)
        except ValueError:
            print("No image found at time: ",time)
            return None,None
        dec_img = self.image_buffer[dec_index]
        try:
            pose_index = self.pose_times.index(time)
        except ValueError:
            print("No poses found at time: ",time)
            if time>self.pose_times[-1]:
                pose_index = -1
            else:
                return dec_img,None
        positions = self.pose_buffer[pose_index]
        return dec_img,positions
    
    def ros_img_to_cv2(self,ros_img):
        try:
            return self.cv_bridge.imgmsg_to_cv2(ros_img, "bgr8")
        except CvBridgeError:
            raise CvBridgeError


    def draw_labels(self,img,positions,ids):
        if img is None or positions is None:
            return img
            
        
        orgs = []
        names = []
        for i in range(len(positions.bodies)):
            if positions.points2d[i].x == -1 or positions.points2d[i].y == -1:
                continue
            names.append(ids[positions.bodies[i]])
            orgs.append((int(img.shape[1]*positions.points2d[i].x),int(img.shape[0]*positions.points2d[i].y)))
        
        # Draw
        new_img = self.draw_text(img,names,orgs)

        return new_img
    
    def draw_text(self,img,texts,positions,radius=32,text_size=2,text_thickness=4):
        # Draw circle
        shapes = np.zeros_like(img, np.uint8)

        for position in positions:
            cv2.circle(shapes, position, radius, (255,255,255), -1, lineType=8)

        # Blend
        out = img.copy()
        alpha = 0.5
        mask = shapes.astype(bool)
        out[mask] = cv2.addWeighted(img, alpha, shapes, 1 - alpha, 0)[mask]    

        # Draw label
        for text,position in zip(texts,positions):
            text_size_cv2, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, text_size, text_thickness)
            text_origin = (int(position[0] - text_size_cv2[0] / 2), int(position[1] + text_size_cv2[1] / 2))
            cv2.putText(out, text, text_origin, cv2.FONT_HERSHEY_SIMPLEX ,  text_size, (0,0,255), 4, cv2.LINE_AA)

        return out

    def save_image(self,img):
        cv2.imwrite('/home/tamlin/engage/latest_decision.jpeg', img)

    def save_explainability_test(self,explainability_test):
        f = open("/home/tamlin/engage/explanations.txt", "a")
        # Group
        g_text = "Group: {}\n".format(explainability_test.group)
        f.write(g_text)
        # Explanation
        e_text = "Explanation Reason: {}\n".format(explainability_test.exp_reason)
        f.write(e_text)
        if explainability_test.group == 2:
            c_text = "Explanation Counterfactual: {}\n".format(explainability_test.exp_counterfactual)
            f.write(c_text)
        
        # Question
        q_context = "Question Context: {}\n".format(explainability_test.question_context)
        f.write(q_context)
        q_text = "Question: {}\n".format(explainability_test.question_text)
        f.write(q_text)

        # Answers
        answers = "Answers: "
        for answer,i in zip(explainability_test.answer_texts,["a","b","c"]):
            answers += "({}) {}; ".format(i,answer)
        f.write(answers[:-2]+"\n")

        f.write("-------------------------------------------------------------------------\n")

        f.close()

    def get_name_mapping(self,positions):
        if positions is None:
            return None,None
        
        x_poses = {}
        for i in range(len(positions.bodies)):
            x_poses[positions.bodies[i]] = positions.points2d[i].x
        body_order = sorted(x_poses, key=x_poses.get)
        
        names = {}
        ids = {}
        for i in range(len(body_order)):
            if self.language.lower() in ["english","en_gb"]:
                person_marker = "Person"
            elif self.language.lower() in ["catalan","ca_es"]:
                person_marker = "Persona"
            else:
                raise Exception("Langauge {} not recognised".format(self.language))
            names[body_order[i]] = "{} {}".format(person_marker,string.ascii_uppercase[i])
            ids[body_order[i]] = string.ascii_uppercase[i]

        return names,ids
        
            

    def publish_explainability_test(self,et_test,image,time):
        ros_img = self.cv_bridge.cv2_to_compressed_imgmsg(image)
        blank_image = self.cv_bridge.cv2_to_compressed_imgmsg(np.zeros((1,1,3), np.uint8))
        message = et_test.to_message(ros_img,time,blank_image)
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
    parser.add_argument("-p", "--pose_image_topic", help="Topic for annotated pose image",
                        type=str, default="/opendr/pose_img")
    parser.add_argument("-d", "--decision_maker", help="Which decision maker will be used",
                        type=str, default="heuristic")
    parser.add_argument("--buffer_time", help="How long the node will store images for",
                        type=int, default=5)
    parser.add_argument("--explainer", help="Which explainer will be used",
                        type=str, default="counterfactual")
    parser.add_argument("--groups", help="Which groups to include. all = [0,1,2]. control = [0]. nocf = [1]. full = [2]. nomid = [0,2].",
                        type=str, default="all")
    parser.add_argument("--language", help="Language of the robot, can be 'english' or 'catalan'",
                        type=str, default="english")
    parser.add_argument("--image_mode", help="Whether the image used is rgb or annotated pose",
                        type=str, default="rgb")
    args = parser.parse_args(rospy.myargv()[1:])

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
    
    use_pose = False
    if args.image_mode == "pose":
        use_pose = True

    explainer = LiveExplainer(
        args.rgb_image_topic,
        args.pose_image_topic,
        args.decision_maker,
        buffer_time=args.buffer_time,
        explainer=args.explainer,
        groups=groups,
        language=args.language,
        use_pose=use_pose,
    )
    explainer.run()