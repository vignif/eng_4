import rospy
import numpy as np

from engage.msg import RobotDecision as RobotDecisionMSG,EngagementLevel,MotionActivity
from engage.decision_maker.engage_state import EngageState
from engage.decision_maker.robot_decision import RobotDecision
from engage.decision_maker.decision_maker import DecisionMaker


class RandomRobotDecisionMaker(DecisionMaker):
    def __init__(self,wait_time=5,**kwargs):
        self.wait_time = rospy.Duration(wait_time)

        self.last_decision_time = None

    '''
    DECISION
    '''

    decision = RobotDecision
        
    def update_last_decision_time(self,decision:RobotDecision,time):
        self.last_decision_time = time

    def decide(self,state:EngageState):
        random_gesture = np.random.choice(range(len(RobotDecision.gesture_names)))
        random_gaze = np.random.choice(range(len(RobotDecision.gaze_names)))
        random_speech = np.random.choice(range(len(RobotDecision.speech_names)))
        random_target = np.random.choice(state.bodies+[None])

        return RobotDecision(random_gesture,random_gaze,random_speech,random_target)