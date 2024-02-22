import rospy
import argparse

from engage.decision_maker.decision_manager import DecisionManager
from engage.srv import ToggleInteraction

from engage.msg import DecisionState

class PredictionExperimentManager:
    def __init__(self,
                 decision_maker="heuristic",
                 timeout_duration=10,
                 decision_threshold=1,
                 rate=20):
        self.rate = rospy.Rate(rate)
        self.dt = decision_threshold
        self.timeout_duration = timeout_duration
        self.dm_name = decision_maker

        self.state_type = DecisionManager.decision_state_msgs[self.dm_name]

        # Subscribers
        dec_state_sub = rospy.Subscriber("/hri_engage/decision_states",self.state_type,callback=self.process_decision_state)

        # States
        self.state = "ELICIT"

        # Timer
        self.timeout_timer = None

    def process_decision_state(self,dec_state):
        if self.state == "ELICIT":
            if DecisionManager.state_msgs[self.dm_name] == DecisionState:
                # Dealing with the engage state
                distances = dec_state.state.distances
                for dist in distances:
                    if dist < self.dt:
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
                self.timeout_timer = rospy.Timer(rospy.Duration(10), self.timeout, oneshot=True)
                self.state = new_state
            except:
                print("Failed to toggle interaction")
        elif old_state == "TEST" and new_state == "ELICIT":
            rospy.wait_for_service('toggle_interaction')
            try:
                toggle_interaction = rospy.ServiceProxy('toggle_interaction', ToggleInteraction)
                toggle_interaction(True)
                self.state = new_state
            except:
                self.timeout_timer = rospy.Timer(rospy.Duration(1), self.timeout, oneshot=True)
                print("Failed to toggle interaction")


        print("State changed from {} to {}".format(old_state,self.state))

    def timeout(self,timer):
        # Timeout triggered, go back to interacting
        self.change_state("ELICIT")


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
    parser.add_argument("-t", "--timeout_duration", help="Length of inactivity time before timeout",
                        type=float, default=10)
    args = parser.parse_args(rospy.myargv()[1:])

    explainer = PredictionExperimentManager(
        decision_maker=args.decision_maker,
        decision_threshold=args.decision_threshold,
        timeout_duration=args.timeout_duration,
    )
    explainer.run()