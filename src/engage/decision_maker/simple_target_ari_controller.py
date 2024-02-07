import rospy
import numpy as np
import tf
import copy

from play_motion_msgs.msg import PlayMotionActionGoal
from pal_interaction_msgs.msg import TtsActionGoal
from geometry_msgs.msg import PointStamped
from engage.msg import RobotDecision as RobotDecisionMSG
from engage.msg import HeuristicDecision as HeuristicDecisionMSG
from engage.decision_maker.decision_maker import RobotController
from engage.decision_maker.robot_decision import RobotDecision
from engage.decision_maker.heuristic_decision import HeuristicDecision
from engage.decision_maker.engage_state import EngageState

class SimpleTargetARIController(RobotController):

    def __init__(self,world_frame="base_link",z_offset=0.3,**kwargs) -> None:
        self.world_frame = world_frame
        # Parameters
        self.z_offset = z_offset
        # Publishers
        self.motion_action_publisher = rospy.Publisher("/play_motion/goal",PlayMotionActionGoal,queue_size=1)
        self.gaze_action_publisher = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.tts_publisher = rospy.Publisher("/tts/goal",TtsActionGoal,queue_size=1)

        # DEBUG TODO
        self.listener = tf.TransformListener()

    def execute_command(self,decision:HeuristicDecision,state:EngageState,bodies=None):
        if decision.action == HeuristicDecisionMSG.NOTHING:
            # Gaze ahead
            gaze = PointStamped()
            gaze.header.frame_id = self.world_frame
            gaze.point.x = 1
            gaze.point.y = 0
            gaze.point.z = 1.5
            self.gaze_action_publisher.publish(gaze)
        elif decision.action == HeuristicDecisionMSG.WAIT:
            return None
        elif decision.action == HeuristicDecisionMSG.ELICIT_TARGET:
            '''
            gaze = PointStamped()
            gaze.header.frame_id = "{}_face_tf".format(decision.target)
            gaze.point.x = 0
            gaze.point.y = 0
            gaze.point.z = 0
            self.gaze_action_publisher.publish(gaze)
            '''
            gaze = PointStamped()
            gaze.header.frame_id = self.world_frame
            
            
            if decision.target in bodies and bodies[decision.target] is not None and bodies[decision.target].position is not None:
                target_pos = copy.deepcopy(bodies[decision.target].position)
                if self.world_frame == "base_link":
                # The following changes are necessary for some reason to properly convert to the base_link for the ARI robot
                    target_pos.position.x = abs(target_pos.position.x)
                    target_pos.position.y = -target_pos.position.y
                target_pos.position.z += self.z_offset

                # Limits on z
                target_pos.position.z = min(target_pos.position.z,1.75)
                target_pos.position.z = max(target_pos.position.z,0.5)
                
                gaze.point = target_pos.position
                print(gaze.point)
                self.gaze_action_publisher.publish(gaze)
            else:
                return None
        else:
            # Gaze ahead
            gaze = PointStamped()
            gaze.header.frame_id = self.world_frame
            gaze.point.x = 1
            gaze.point.y = 0
            gaze.point.z = 1.5
            self.gaze_action_publisher.publish(gaze)
