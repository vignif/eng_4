import rospy
import numpy as np

from play_motion_msgs.msg import PlayMotionActionGoal
from pal_interaction_msgs.msg import TtsActionGoal
from geometry_msgs.msg import PointStamped
from engage.msg import RobotDecision as RobotDecisionMSG
from engage.decision_maker.decision_maker import RobotController
from engage.decision_maker.robot_decision import RobotDecision
from engage.decision_maker.engage_state import EngageState

class SimpleARIController(RobotController):
    def __init__(self,world_frame="base_link") -> None:
        self.world_frame = world_frame
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
                motion = "wave"
            elif decision.gesture == RobotDecisionMSG.GESTURE_ALIVE:
                motion = "alive_6"
            
            if motion is not None:
                motion_msg = PlayMotionActionGoal()
                motion_msg.goal.motion_name = motion
                self.motion_action_publisher.publish(motion_msg)

            # Gaze
            gaze = PointStamped()
            gaze.header.stamp = state.time

            target = decision.target
            target_pos = None
            if target is not None and bodies is not None and target in bodies:
                target_pos = bodies[target].position

            if target_pos is not None:
                # Look at target
                gaze.header.frame_id = self.world_frame
                gaze.point = target_pos.position
            else:
                # Gaze ahead
                gaze.header.frame_id = "base_link"
                gaze.point.x = 1
                gaze.point.y = 0
                gaze.point.z = 1.5
            '''
            if decision.gaze == RobotDecisionMSG.GAZE_AHEAD:
                gaze.header.frame_id = "sellion_link"
                gaze.point.x = 10
                gaze.point.y = 0
                gaze.point.z = 0
            elif decision.gaze == RobotDecisionMSG.GAZE_TARGET:
                target = decision.target
                target_pos = None
                if target is not None and bodies is not None and target in bodies:
                    target_pos = bodies[target].position

                if target_pos is not None:
                    # Look at target
                    gaze.header.frame_id = self.world_frame
                    gaze.point = target_pos.position
                else:
                    # Gaze ahead
                    gaze.header.frame_id = "sellion_link"
                    gaze.point.x = 10
                    gaze.point.y = 0
                    gaze.point.z = 0
            '''
            self.gaze_action_publisher.publish(gaze)
                
            # Speech
            speech = None
            if decision.speech == RobotDecisionMSG.SPEECH_GREETING_INFORMAL:
                speech = "Hi!"
            elif decision.speech == RobotDecisionMSG.SPEECH_GREETING_FORMAL:
                speech = "Good day"
            elif decision.speech == RobotDecisionMSG.SPEECH_BECKON_ROBOT:
                speech = "Come and play with me!"
            elif decision.speech == RobotDecisionMSG.SPEECH_BECKON_TABLET:
                speech = "Come and see what's on my tablet!"
            elif decision.speech == RobotDecisionMSG.SPEECH_RECAPTURE:
                speech = "Are you leaving so soon?"

            if speech is not None:
                tts_msg = TtsActionGoal()
                tts_msg.goal.rawtext.lang_id = "en_gb"
                tts_msg.goal.rawtext.text = speech
                self.tts_publisher.publish(tts_msg)

