import rospy

from engage.msg import RobotDecision as RobotDecisionMSG
from engage.decision_maker.decision_maker import Decision

class RobotDecision(Decision):
    def __init__(self,gesture:int,gaze:int,speech:int,target:str):
        self.gesture = gesture
        self.gaze = gaze
        self.speech = speech
        self.target = target

    def message(self,time=None):
        msg = RobotDecision()
        if time is not None:
            msg.header.stamp = time
        else:
            msg.header.stamp = rospy.Time.now()
        msg.gesture = self.gesture
        msg.gaze = self.gaze
        msg.speech = self.speech
        msg.target = self.target
        return msg
    
    @staticmethod
    def create_publisher(topic="/hri_engage/decisions",queue_size=1):
        return rospy.Publisher(topic,RobotDecisionMSG,queue_size=queue_size)



    