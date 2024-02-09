import rospy
import argparse

from sensor_msgs.msg import Image

from engage.decision_maker.decision_manager import DecisionManager

class LiveExplainer:
    def __init__(self,
                 rgb_image_topic,
                 decision_maker,
                 buffer_time=5,
                 rate=20) -> None:
        self.rate = rospy.Rate(rate)
        self.decision_maker = decision_maker
        self.buffer_length = rate*buffer_time

        self.dm = DecisionManager.decision_makers[decision_maker]

        # Subscribers
        rgb_img_sub = rospy.Subscriber(rgb_image_topic,Image,callback=self.update_image_buffer)
        dec_sub = rospy.Subscriber(
            "/hri_engage/decision_states",
            DecisionManager.decision_state_msgs[decision_maker],
            callback=self.decision_state_callback
            )
        
        # Image buffer
        self.image_buffer = []
        self.image_times = []

    def update_image_buffer(self,rgb_img):
        self.image_buffer.append(rgb_img)
        self.image_times.append(rgb_img.header.stamp)

        if len(self.image_buffer)>self.buffer_length:
            self.image_buffer = self.image_buffer[-self.buffer_length:]
            self.image_times = self.image_times[-self.buffer_length:]

    def decision_state_callback(self,dec):
        dec_time = dec.header.stamp

        if self.dm.decision.interesting_decision(dec.decision):
            # Only handle decisions that are interesting, not NOTHING or WAITING
            try:
                dec_index = self.image_times.index(dec_time)
            except ValueError:
                print("No image found at time: ",dec_time)
                return None
            
            dec_img = self.image_buffer[dec_index]

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
    args = parser.parse_args()

    explainer = LiveExplainer(
        args.rgb_image_topic,
        args.decision_maker,
        buffer_time=args.buffer_time,
    )
    explainer.run()