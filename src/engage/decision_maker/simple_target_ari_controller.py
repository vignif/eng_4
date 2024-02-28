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
from hri_msgs.msg import Expression

class SimpleTargetARIController(RobotController):

    elicit_target_speech = {
        "en_GB": ["Hi","Hello","Come and talk with me","Come and play a game!","Do you want to talk with me?","Do you want to play a game?"],
        #"ca_ES": ["Hola","Bon dia","Apropa't i parla amb mi","Apropa't i juga amb mi!","Vols parlar amb mi?","Vols jugar un joc?"],
        "ca_ES":["Hola","Bon dia","Com estàs?","Ei!"]
    }

    elicit_general_speech = {
        "en_GB": ["Does anybody want to talk with me?","Come and see what's on my tablet"],
        #"ca_ES": ["Algú vol parlar amb mi?","Apropa't i mira la meva tauleta"]
        "ca_ES":["Hola","Bon dia","Com estàs?","Ei!"]
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
        self.eye_publisher = rospy.Publisher("/robot_face/expression",Expression,queue_size=1)

        self.alive_motions = ["alive_{}".format(i) for i in [1,2,5,6]]

    def execute_command(self,decision:HeuristicDecision,state:EngageState,bodies=None):
        if decision.action == HeuristicDecisionMSG.NOTHING:
            self.execute_nothing()
        elif decision.action == HeuristicDecisionMSG.WAIT:
            self.execute_wait()
        elif decision.action == HeuristicDecisionMSG.ELICIT_TARGET:
            self.execute_elicit_target(decision,state,bodies)
        elif decision.action == HeuristicDecisionMSG.ELICIT_GENERAL:
            self.execute_elicit_general()
        else:
            self.execute_nothing()

    def execute_nothing(self):
        # Gaze ahead
        gaze = PointStamped()
        gaze.header.frame_id = self.world_frame
        gaze.point.x = 1
        gaze.point.y = 0
        gaze.point.z = 1.5
        self.gaze_action_publisher.publish(gaze)
        # Eyes
        expression = Expression()
        expression.expression = "asleep"
        self.eye_publisher.publish(expression)

    def execute_wait(self):
        return None
    
    def execute_elicit_target(self,decision:HeuristicDecision,state:EngageState,bodies=None):
        # Gaze at target
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
            self.gaze_action_publisher.publish(gaze)
        
        # Eyes
        expression = Expression()
        expression.expression = "neutral"
        self.eye_publisher.publish(expression)

        # Gesture
        motion_msg = PlayMotionActionGoal()
        motion_msg.goal.motion_name = np.random.choice(["wave","wave_left"])
        self.motion_action_publisher.publish(motion_msg)

        # Speech
        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.lang_id = self.lang_id
        tts_msg.goal.rawtext.text = np.random.choice(self.elicit_target_speech[self.lang_id])
        self.tts_publisher.publish(tts_msg)

        
    def execute_elicit_general(self):
        # Gaze
        self.reset_gaze()

        # Eyes
        expression = Expression()
        expression.expression = "bored"
        self.eye_publisher.publish(expression)

        # Gesture
        motion_msg = PlayMotionActionGoal()
        motion_msg.goal.motion_name = np.random.choice(self.alive_motions)
        self.motion_action_publisher.publish(motion_msg)

        # Speech
        tts_msg = TtsActionGoal()
        tts_msg.goal.rawtext.lang_id = self.lang_id
        tts_msg.goal.rawtext.text = np.random.choice(self.elicit_general_speech[self.lang_id])
        self.tts_publisher.publish(tts_msg)

    def reset_gaze(self):
        gaze = PointStamped()
        gaze.header.frame_id = "sellion_link"
        gaze.point.x = 10
        gaze.point.y = 0
        gaze.point.z = 0
        self.gaze_action_publisher.publish(gaze)

    def execute_start_tablet_behaviour(self):
        # Gaze ahead
        self.reset_gaze()
        # Eyes
        expression = Expression()
        expression.expression = "excited"
        self.eye_publisher.publish(expression)

    def execute_new_page_behaviour(self,page_name):
        # Gaze ahead
        self.reset_gaze()

        if page_name == "welcome_page":
            exp = "excited"
        elif page_name == "consent_form_page":
            exp = "neutral"
        elif page_name == "played_before_page":
            exp = "neutral"
        elif page_name == "explanation_page":
            exp = "neutral" 
        elif page_name == "prediction_page":
            exp = "confused" 
        elif page_name == "end_page":
            exp = "amazed"

        expression = Expression()
        expression.expression = exp
        self.eye_publisher.publish(expression)