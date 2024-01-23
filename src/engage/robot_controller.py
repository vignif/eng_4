import rospy

from play_motion_msgs.msg import PlayMotionActionGoal
from pal_interaction_msgs.msg import TtsActionGoal
from geometry_msgs.msg import PointStamped
from engage.msg import Decision

class SimpleARIController:
    def __init__(self,world_frame="map") -> None:
        self.world_frame = world_frame
        # Publishers
        self.motion_action_publisher = rospy.Publisher("/play_motion/goal",PlayMotionActionGoal,queue_size=1)
        self.gaze_action_publisher = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.tts_publisher = rospy.Publisher("/tts/goal",TtsActionGoal,queue_size=1)

    def execute_command(self,target,action,bodies):
        if action == Decision.NOTHING or action == Decision.WAIT:
            return None
        elif action == Decision.ELICIT_GENERAL:
            motion = "wave"
        elif action == Decision.ELICIT_TARGET:
            motion = "bow"
        else:
            motion = "flying"
        motion_msg = PlayMotionActionGoal()
        motion_msg.goal.motion_name = motion
        self.motion_action_publisher.publish(motion_msg)

        if target is not None:
            target_body = bodies[target]
            if target_body.position is not None:
                target_pos = PointStamped()
                target_pos.header.frame_id = "sellion_link"
                target_pos.header.stamp = rospy.Time.now()
                target_pos.point.x = target_body.position.position.x
                target_pos.point.y = target_body.position.position.y
                target_pos.point.z = target_body.position.position.z
                print(target_pos)
                self.gaze_action_publisher.publish(target_pos)

        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.lang_id = "en_gb"
        if action == Decision.ELICIT_GENERAL:
            text = "Hello! Does anyone want to talk with me?"
        elif action == Decision.ELICIT_TARGET:
            text = "Hello there. Do you want to talk?"
        elif action == Decision.RECAPTURE:
            text = "Wait! Don't leave me!"
        elif action == Decision.MAINTAIN:
            text = "I'm so glad you're talking with me"

        tts_msg.goal.rawtext.text = text
        self.tts_publisher.publish(tts_msg)