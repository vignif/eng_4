import rospy
import numpy as np
import tf

from play_motion_msgs.msg import PlayMotionActionGoal
from pal_interaction_msgs.msg import TtsActionGoal
from geometry_msgs.msg import PointStamped
from engage.msg import RobotDecision as RobotDecisionMSG
from engage.decision_maker.decision_maker import RobotController
from engage.decision_maker.robot_decision import RobotDecision
from engage.decision_maker.engage_state import EngageState

class SimpleARIController(RobotController):

    informal_greeting = {
        "en_GB":"Hi!",
        "ca_ES":"Hola!",
    }

    formal_greeting = {
        "en_GB":"Hello",
        "ca_ES":"Bon dia",
    }

    beckon_robot = {
        "en_GB":"Come and talk with me",
        "ca_ES":"Apropa't i parla amb mi",
    }

    beckon_tablet = {
        "en_GB":"Come and see what's on my tablet",
        "ca_ES":"Apropa't i mira la meva tauleta",
    }

    recapture = {
        "en_GB":"Are you leaving so soon?",
        "ca_ES":"Ja marxes?",
    }

    def __init__(self,world_frame="base_link",z_offset=0.3,language="english",**kwargs) -> None:
        self.world_frame = world_frame
        # Parameters
        self.z_offset = z_offset
        if language == "english" or language == "en_GB":
            self.lang_id = "en_GB"
        elif language == "catalan" or language == "ca_ES":
            self.lang_id = "ca_ES"
        else:
            error_message = "Cannot identify language: {}, must be english or catalan".format(language)
            raise Exception(error_message)
        # Publishers
        self.motion_action_publisher = rospy.Publisher("/play_motion/goal",PlayMotionActionGoal,queue_size=1)
        self.gaze_action_publisher = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.tts_publisher = rospy.Publisher("/tts/goal",TtsActionGoal,queue_size=1)

    def execute_command(self,decision:RobotDecision,state:EngageState,bodies=None):
        if decision.wait:
            return None
        else:
            # Gesture
            motion = None
            if decision.gesture == RobotDecisionMSG.GESTURE_WAVE:
                motion = np.random.choice(["wave","wave_left"])
            elif decision.gesture == RobotDecisionMSG.GESTURE_ALIVE:
                motion = "alive_6"
            
            if motion is not None:
                motion_msg = PlayMotionActionGoal()
                motion_msg.goal.motion_name = motion
                self.motion_action_publisher.publish(motion_msg)

            # Gaze
            gaze = PointStamped()

            if decision.gaze == RobotDecisionMSG.GAZE_AHEAD:
                # Gaze ahead
                gaze.header.frame_id = self.world_frame
                gaze.point.x = 1
                gaze.point.y = 0
                gaze.point.z = 1.5
            elif decision.gaze == RobotDecisionMSG.GAZE_TARGET:
                target = decision.target
                target_pos = None
                if target is not None and bodies is not None and target in bodies:
                    target_pos = bodies[target].position

                if target_pos is not None:
                    # Look at target
                    gaze.header.frame_id = self.world_frame

                    
                    gaze.point = target_pos.position
                    if self.world_frame == "base_link":
                    # The following changes are necessary for some reason to properly convert to the base_link for the ARI robot
                        target_pos.position.x = abs(target_pos.position.x)
                        target_pos.position.y = -target_pos.position.y
                    target_pos.position.z += self.z_offset
                else:
                    # Gaze ahead
                    # TODO: Maybe a random target instead?
                    gaze.header.frame_id = self.world_frame
                    gaze.point.x = 1
                    gaze.point.y = 0
                    gaze.point.z = 1.5
            self.gaze_action_publisher.publish(gaze)
                
            # Speech
            speech = None
            if decision.speech == RobotDecisionMSG.SPEECH_GREETING_INFORMAL:
                speech = self.informal_greeting[self.lang_id]
            elif decision.speech == RobotDecisionMSG.SPEECH_GREETING_FORMAL:
                speech = self.formal_greeting[self.lang_id]
            elif decision.speech == RobotDecisionMSG.SPEECH_BECKON_ROBOT:
                speech = self.beckon_robot[self.lang_id]
            elif decision.speech == RobotDecisionMSG.SPEECH_BECKON_TABLET:
                speech = self.beckon_tablet[self.lang_id]
            elif decision.speech == RobotDecisionMSG.SPEECH_RECAPTURE:
                speech = self.recapture[self.lang_id]

            if speech is not None:
                tts_msg = TtsActionGoal()
                tts_msg.goal.rawtext.lang_id = self.lang_id
                tts_msg.goal.rawtext.text = speech
                self.tts_publisher.publish(tts_msg)

