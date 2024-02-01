import rospy
import numpy as np

from play_motion_msgs.msg import PlayMotionActionGoal
from pal_interaction_msgs.msg import TtsActionGoal
from geometry_msgs.msg import PointStamped
from engage.msg import HeuristicDecision as HeuristicDecisionMSG
from engage.decision_maker.decision_maker import RobotController
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.engage_state import EngageState

class HeuristicARIController(RobotController):
    def __init__(self,world_frame="map") -> None:
        self.world_frame = world_frame
        # Publishers
        self.motion_action_publisher = rospy.Publisher("/play_motion/goal",PlayMotionActionGoal,queue_size=1)
        self.gaze_action_publisher = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.tts_publisher = rospy.Publisher("/tts/goal",TtsActionGoal,queue_size=1)

    def execute_command(self,decision:HeuristicDecision,state:EngageState,bodies=None):
        if decision.action == HeuristicDecisionMSG.NOTHING or decision.action == HeuristicDecisionMSG.WAIT:
            return None
        elif decision.action == HeuristicDecisionMSG.ELICIT_GENERAL:
            motion = "wave"
        elif decision.action == HeuristicDecisionMSG.ELICIT_TARGET:
            motion = "bow"
        else:
            motion = "flying"
        motion_msg = PlayMotionActionGoal()
        motion_msg.goal.motion_name = motion
        self.motion_action_publisher.publish(motion_msg)

        target_pos = PointStamped()
        target_pos.header.frame_id = "sellion_link"
        #target_pos.header.stamp = rospy.Time.now()
        target_pos.header.stamp = state.time
        y = np.random.choice([-0.1,0,0.1])
        z = np.random.choice([-1,0,1])
        target_pos.point.x = 1
        target_pos.point.y = y
        target_pos.point.z = z
        print(target_pos)
        self.gaze_action_publisher.publish(target_pos)

        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.lang_id = "en_gb"
        if decision.action == HeuristicDecisionMSG.ELICIT_GENERAL:
            text = "Hello! Does anyone want to talk with me?"
        elif decision.action == HeuristicDecisionMSG.ELICIT_TARGET:
            text = "Hello there. Do you want to talk?"
        elif decision.action == HeuristicDecisionMSG.RECAPTURE:
            text = "Wait! Don't leave me!"
        elif decision.action == HeuristicDecisionMSG.MAINTAIN:
            text = "I'm so glad you're talking with me"

        tts_msg.goal.rawtext.text = text
        self.tts_publisher.publish(tts_msg)