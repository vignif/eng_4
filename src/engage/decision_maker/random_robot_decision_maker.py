import rospy
import numpy as np

from engage.msg import RobotDecision as RobotDecisionMSG
from engage.decision_maker.engage_state import EngageState
from engage.decision_maker.robot_decision import RobotDecision
from engage.decision_maker.decision_maker import DecisionMaker


class RandomRobotDecisionMaker(DecisionMaker):
    def __init__(self,wait_time=5,wait_deviation=1,reduced_action_space=False,**kwargs):
        self.wait_time = rospy.Duration(wait_time)
        self.wait_mean = wait_time
        self.wait_deviation = wait_deviation

        self.last_decision_time = None
        self.curr_wait_time = None
        self.reduced_action_space = reduced_action_space

        if self.reduced_action_space:
            # Reduced action space
            self.allowed_gestures = [
                RobotDecisionMSG.GESTURE_NOTHING,
                RobotDecisionMSG.GESTURE_WAVE
            ]

            self.allowed_gazes = range(len(RobotDecision.gaze_names))

            self.allowed_speeches = [
                RobotDecisionMSG.SPEECH_NOTHING,
                RobotDecisionMSG.SPEECH_BECKON_ROBOT
            ]
        else:
            # Full action space
            self.allowed_gestures = range(len(RobotDecision.gesture_names))
            self.allowed_gazes = range(len(RobotDecision.gaze_names))
            self.allowed_speeches = range(len(RobotDecision.speech_names))


    '''
    DECISION
    '''

    decision = RobotDecision
        
    def update_last_decision_time(self,decision:RobotDecision,time):
        if not decision.wait:
            self.last_decision_time = time
            # New wait time
            if self.wait_deviation is not None and self.wait_deviation != 0:
                self.wait_time = rospy.Duration(np.random.normal(self.wait_mean,self.wait_deviation))


    def decide(self,state:EngageState):
        if state.waiting:
            return RobotDecision(
                True,
                RobotDecisionMSG.GESTURE_NOTHING,
                RobotDecisionMSG.GAZE_AHEAD,
                RobotDecisionMSG.SPEECH_NOTHING,
                None,
            )
        else:
            random_gesture = np.random.choice(self.allowed_gestures)
            random_gaze = np.random.choice(self.allowed_gazes)
            random_speech = np.random.choice(self.allowed_speeches)
            random_target = np.random.choice(state.bodies+[None])

            return RobotDecision(False,random_gesture,random_gaze,random_speech,random_target)