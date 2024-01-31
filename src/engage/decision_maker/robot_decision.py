import rospy

from engage.msg import RobotDecision as RobotDecisionMSG
from engage.decision_maker.decision_maker import Decision

class RobotDecision(Decision):
    gesture_names = ["GESTURE_NOTHING","GESTURE_WAVE","GESTURE_ALIVE"]
    gaze_names = ["GAZE_AHEAD","GAZE_TARGET"]
    speech_names = ["SPEECH_NOTHING",
                    "SPEECH_GREETING_INFORMAL",
                    "SPEECH_GREETING_FORMAL",
                    "SPEECH_BECKON_ROBOT",
                    "SPEECH_BECKON_TABLET",
                    "SPEECH_RECAPTURE",
                    ]

    def __init__(self,gesture:int,gaze:int,speech:int,target:str):
        self.gesture = gesture
        self.gaze = gaze
        self.speech = speech
        self.target = target

    def message(self,time=None):
        msg = RobotDecisionMSG()
        if time is not None:
            msg.header.stamp = time
        else:
            msg.header.stamp = rospy.Time.now()
        msg.gesture = self.gesture
        msg.gaze = self.gaze
        msg.speech = self.speech
        msg.target = self.target if self.target is not None else ""
        return msg
    
    def decision_tuple(self):
        return (self.gesture,self.gaze,self.speech,self.target)
    
    def decision_tuple_string(self):
        return (
            self.gesture_names[self.gesture],
            self.gaze_names[self.gaze],
            self.speech_names[self.speech],
            self.target
            )
    
    @staticmethod
    def create_publisher(topic="/hri_engage/decisions",queue_size=1):
        return rospy.Publisher(topic,RobotDecisionMSG,queue_size=queue_size)



    