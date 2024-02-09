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
    
    component_names = ["Wait","Gesture","Gaze","Speech","Target"]

    def __init__(self,wait:bool,gesture:int,gaze:int,speech:int,target:str):
        self.wait = wait
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
        msg.wait = self.wait
        msg.gesture = self.gesture
        msg.gaze = self.gaze
        msg.speech = self.speech
        msg.target = self.target if self.target is not None else ""
        return msg
    
    def decision_tuple(self):
        return (self.wait,self.gesture,self.gaze,self.speech,self.target)
    
    def decision_tuple_string(self):
        return (
            str(int(self.wait)),
            str(self.gesture),
            str(self.gaze),
            str(self.speech),
            self.target
        )
    
    def decision_string(self):
        return "{}_{}_{}_{}_{}".format(*self.decision_tuple_string())
    
    @staticmethod
    def update_state_decision(states,decision):
        states["DECISION"]["Wait"].append(decision.wait)
        states["DECISION"]["Gesture"].append(decision.gesture)
        states["DECISION"]["Gaze"].append(decision.gaze)
        states["DECISION"]["Speech"].append(decision.speech)
        states["DECISION"]["Target"].append(decision.target)

        return states
    
    @staticmethod
    def create_publisher(topic="/hri_engage/decisions",queue_size=1):
        return rospy.Publisher(topic,RobotDecisionMSG,queue_size=queue_size)
    
    @staticmethod
    def interesting_decision(decision_message):       
        return not decision_message.wait



    