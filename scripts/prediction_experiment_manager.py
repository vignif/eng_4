import rospy
import argparse

from engage.decision_maker.decision_manager import DecisionManager
from engage.srv import ToggleInteraction

from engage.msg import DecisionState
from std_msgs.msg import String
from pal_interaction_msgs.msg import Input
from pal_web_msgs.msg import WebGoTo

class PredictionExperimentManager:
    def __init__(self,
                 decision_maker="heuristic",
                 start_timeout_duration=15,
                 input_timeout_duration=30,
                 decision_threshold=1,
                 robot_controller="heuristic_ari_controller",
                 world_frame="base_link",
                 rate=20,
                 **kwargs):
        self.rate = rospy.Rate(rate)
        self.dt = decision_threshold
        self.start_timeout_duration = start_timeout_duration
        self.input_timeout_duration = input_timeout_duration
        self.dm_name = decision_maker

        self.state_type = DecisionManager.decision_state_msgs[self.dm_name]

        # Subscribers
        dec_state_sub = rospy.Subscriber("/hri_engage/decision_states",self.state_type,callback=self.process_decision_state)
        page_name_sub = rospy.Subscriber("/page_name",String,self.page_name_callback)
        user_input_sub = rospy.Subscriber("/user_input",Input,self.user_input_callback)

        # Publishers
        self.page_pub = rospy.Publisher("/web/go_to",WebGoTo,queue_size=1)

        # States
        self.state = "ELICIT"

        # Timer
        self.timeout_timer = None

        # Robot controller
        self.robot_controller = self.robot_controller = DecisionManager.robot_controllers[robot_controller](world_frame=world_frame,**kwargs)

        # Welcome page
        self.page = None
        self.navigate_page("welcome_page")

    def process_decision_state(self,dec_state):
        if self.state == "ELICIT":
            if DecisionManager.state_msgs[self.dm_name] == DecisionState:
                # Dealing with the engage state
                distances = dec_state.state.distances
                for dist in distances:
                    if dist < self.dt and dist != 0: # != 0 because sometimes depth will fail
                        # Change state
                        self.change_state("TEST")
                        break

    def change_state(self,new_state):
        old_state = self.state

        if old_state == "ELICIT" and new_state == "TEST":
            rospy.wait_for_service('toggle_interaction')
            try:
                toggle_interaction = rospy.ServiceProxy('toggle_interaction', ToggleInteraction)
                toggle_interaction(False)
                self.timeout_timer = rospy.Timer(rospy.Duration(self.start_timeout_duration), self.timeout, oneshot=True)
                self.state = new_state
            except:
                print("Failed to toggle interaction")
            else:
                self.robot_controller.execute_start_tablet_behaviour()
        elif old_state == "TEST" and new_state == "ELICIT":
            rospy.wait_for_service('toggle_interaction')
            try:
                toggle_interaction = rospy.ServiceProxy('toggle_interaction', ToggleInteraction)
                toggle_interaction(True)
                self.navigate_page("welcome_page")
                self.state = new_state
            except:
                self.timeout_timer = rospy.Timer(rospy.Duration(1), self.timeout, oneshot=True)
                print("Failed to toggle interaction")


        print("State changed from {} to {}".format(old_state,self.state))

    def page_name_callback(self,page_name:String):
        self.page = page_name.data
        self.robot_controller.execute_new_page_behaviour(page_name.data)

    def user_input_callback(self,user_input):
        print("Reset timer")
        if self.state == "TEST":
            if self.timeout_timer is not None:
                self.timeout_timer.shutdown()
                if self.page in ["prediction_page","end_page"]:
                    duration = self.start_timeout_duration
                else:
                    duration = self.input_timeout_duration
                self.timeout_timer = rospy.Timer(rospy.Duration(duration), self.timeout, oneshot=True)
            # Lock decision making just in case it isn't already locked
            try:
                toggle_interaction = rospy.ServiceProxy('toggle_interaction', ToggleInteraction)
                toggle_interaction(False)
            except:
                print("Failed to toggle interaction")
        elif self.state == "ELICIT":
            self.change_state("TEST")

    def timeout(self,timer):
        # Timeout triggered, go back to interacting
        print("Timeout")
        self.change_state("ELICIT")

    def navigate_page(self,page_name):
        go_to = WebGoTo()
        go_to.type = WebGoTo.TOUCH_PAGE
        go_to.value = page_name
        self.page_pub.publish(go_to)
        self.page = page_name

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("HRIPredictionExperimentManager", anonymous=True)

    default_camera = "camera"
    parser = argparse.ArgumentParser(description="Manage the predictions experiment")
    parser.add_argument("-d", "--decision_maker", help="Which decision maker will be used",
                        type=str, default="heuristic")
    parser.add_argument("-dt", "--decision_threshold", help="How close someone has to be to stop the decision-making and start the test",
                        type=float, default=1)
    parser.add_argument("--input_timeout_duration", help="Length of inactivity time during test before timeout",
                        type=float, default=30)
    parser.add_argument("--start_timeout_duration", help="Length of inactivity time before test before timeout",
                        type=float, default=15)
    parser.add_argument("-r", "--robot_controller", help="Which robot controller will be used",
                        type=str, default="heuristic_ari_controller")
    parser.add_argument("--language", help="Language of the robot, can be 'english' or 'catalan'",
                        type=str, default="english")
    parser.add_argument("--world_frame", help="World frame",
                        type=str, default="map")
    parser.add_argument("--z_offset", help="Offset to z axis when gazing to account for difference between eye and camera positions",
                        type=float, default=0.3)
    args = parser.parse_args(rospy.myargv()[1:])

    explainer = PredictionExperimentManager(
        decision_maker=args.decision_maker,
        decision_threshold=args.decision_threshold,
        start_timeout_duration=args.start_timeout_duration,
        input_timeout_duration=args.input_timeout_duration,
        robot_controller=args.robot_controller,
        language=args.language,
        world_frame=args.world_frame,
        z_offset=args.z_offset,
    )
    explainer.run()