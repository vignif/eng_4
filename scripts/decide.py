import rospy
import argparse

from message_filters import ApproximateTimeSynchronizer, Subscriber
from engage.msg import EngagementValue,Group,EngagementLevel,MotionActivity,PoseArrayUncertain,HeuristicStateDecision,RobotStateDecision
from hri_msgs.msg import IdsList
from geometry_msgs.msg import TwistStamped,Twist

from engage.decision_maker.heuristic_decision_maker import HeuristicDecisionMaker
from engage.decision_maker.random_robot_decision_maker import RandomRobotDecisionMaker
from engage.decision_maker.simple_target_decision_maker import SimpleTargetDecisionMaker
from engage.pose_helper import HRIPoseBody
from engage.decision_maker.heuristic_ari_controller import HeuristicARIController
from engage.decision_maker.simple_ari_controller import SimpleARIController
from engage.decision_maker.simple_target_ari_controller import SimpleTargetARIController
from engage.decision_maker.engage_state import EngageState
from engage.decision_maker.decision_manager import DecisionManager
from engage.srv import ToggleInteraction,ToggleInteractionResponse,ToggleInteractionRequest

class DecisionBody:
    def __init__(self,id):
        self.id = id

        body_topic = "/humans/bodies/{}/".format(self.id)

        # Subscribers
        self.engagement_level_subscriber = Subscriber(body_topic+"engagement_status",EngagementLevel)
        self.activity_subscriber = Subscriber(body_topic+"activity",MotionActivity)
        self.position_subscriber = rospy.Subscriber("/humans/bodies/{}/poses".format(id),PoseArrayUncertain,self.update_position)
        self.velocity_subscriber = rospy.Subscriber("/humans/bodies/{}/velocity".format(id),TwistStamped,self.update_velocity)

        time_slop = 0.1
        self.synch_sub = ApproximateTimeSynchronizer(
            [
                self.engagement_level_subscriber,
                self.activity_subscriber,
            ],
            1,
            time_slop,
        )
        self.synch_sub.registerCallback(self.update_body)

        # Attributes
        self.engagement_level = EngagementLevel.UNKNOWN
        self.engagement_level_confidence = 0
        self.activity = MotionActivity.NOTHING
        self.activity_confidence = 0
        self.position = None
        self.velocity = None

    def __del__(self):
        self.close()
    
    def close(self):
        self.engagement_level_subscriber.sub.unregister()
        self.activity_subscriber.sub.unregister()
        self.position_subscriber.unregister()
        self.velocity_subscriber.unregister()


    def update_body(self,engagement_level,activity):
        self.engagement_level = engagement_level.level
        self.engagement_level_confidence = engagement_level.confidence
        self.activity = activity.activity
        self.activity_confidence = activity.confidence

    def update_position(self,pose):
        if pose.poses[HRIPoseBody.joints["nose"]] is not None:
            self.position = pose.poses[HRIPoseBody.joints["nose"]]

    def update_velocity(self,vel_msg):
        self.velocity = vel_msg.twist

class DecisionNode:
    def __init__(
            self,
            decision_maker="heuristic",
            robot_controller="heuristic_ari_controller",
            world_frame="map",
            rate=20,
            robot_command=True,
            wait_time=5,
            wait_deviation=1,
            reduced_action_space=False,
            **kwargs
    ):
        # Rate
        self.rate = rospy.Rate(rate)
        self.lock = False

        # Decision-Maker
        self.dm_name = decision_maker
        self.dm = DecisionManager.decision_makers[decision_maker](
            wait_time=wait_time,
            wait_deviation=wait_deviation,
            reduced_action_space = reduced_action_space,
            )

        # Robot Controller
        self.robot_command = robot_command
        if robot_command:
            self.robot_controller = DecisionManager.robot_controllers[robot_controller](world_frame=world_frame,**kwargs)

        # Subscribers
        self.body_subscriber = rospy.Subscriber("/humans/bodies/tracked",IdsList,self.manage_bodies)
        self.engagement_value_subscriber = rospy.Subscriber("/humans/interactions/engagements",EngagementValue,self.update_engagements)
        self.group_subscriber = rospy.Subscriber("/humans/interactions/groups",Group,self.update_groups)

        # Publishers
        self.decision_state_msg = DecisionManager.decision_state_msgs[decision_maker]
        self.decision_publisher = self.dm.decision.create_publisher(topic="/hri_engage/decisions",queue_size=1)
        self.decision_state_publisher = EngageState.create_publisher(self.decision_state_msg,topic="hri_engage/decision_states",queue_size=1)

        # Service
        toggle_service = rospy.Service('toggle_interaction', ToggleInteraction, self.toggle_interaction)

        # Managing bodies
        self.body_time = None
        self.dec_time = None
        self.bodies = {}
        self.groups = {}
        self.group_confidences = {}
        self.distances = {}
        self.engagements = {}
        self.mutual_gazes = {}
        self.pose_confidences = {}

    '''
    CALLBACKS
    '''
    def toggle_interaction(self,tog):
        self.lock = not tog.interacting
        print("Lock set to {}".format(self.lock))
        return ToggleInteractionResponse(True)

    def manage_bodies(self,tracked_msg):
        if self.lock:
            return
        self.lock = True

        self.body_time = tracked_msg.header.stamp
        self.dec_time = rospy.Time.now()
        body_list = tracked_msg.ids

        tracked_ids = set(body_list)
        managed_ids = set(self.bodies.keys())

        # Add new ids
        ids_to_add = list(tracked_ids - managed_ids)
        for id in ids_to_add:
            self.bodies[id] = DecisionBody(id)
            self.groups[id] = None
            self.group_confidences[id] = None
            self.distances[id] = None
            self.engagements[id] = None
            self.mutual_gazes[id] = None
            self.pose_confidences[id] = None
        if "ROBOT" not in self.groups:
            self.groups["ROBOT"] = None
            self.group_confidences["ROBOT"] = None
        
        # Remove old bodies
        ids_to_rem = list(managed_ids - tracked_ids)
        for id in ids_to_rem:
            self.bodies[id].close()
            del self.bodies[id]
            del self.groups[id]
            del self.group_confidences[id]
            del self.distances[id]
            del self.engagements[id]
            del self.mutual_gazes[id]
            del self.pose_confidences[id]

        # Make decision
        self.state = EngageState()
        self.state.node_to_state(
            self.body_time,
            self.bodies,
            self.groups,
            self.group_confidences,
            self.distances,
            self.engagements,
            self.mutual_gazes,
            self.pose_confidences,
            self.dm.is_waiting(self.body_time)
        )


        decision = self.dm.decide(self.state)
        self.dm.update_last_decision_time(decision,self.body_time)

        # Publish decision
        self.decision_publisher.publish(decision.message(time=self.body_time))

        # Publish state
        self.decision_state_publisher.publish(self.state.message(decision,self.decision_state_msg,self.dm_name))

        # Control robot
        if self.robot_command:
            self.robot_controller.execute_command(decision,self.state,self.bodies)

        # Unlock
        self.lock = False

    def update_engagements(self,engagement_value):
        self.ev_time = engagement_value.header.stamp
        if engagement_value.person_a == "" or engagement_value.person_b == "":
            # Involves robot
            if engagement_value.person_a == "":
                id = engagement_value.person_b
                self.pose_confidences[id] = engagement_value.confidence_b
            elif engagement_value.person_b == "":
                id = engagement_value.person_a
                self.pose_confidences[id] = engagement_value.confidence_a
            
            self.distances[id] = engagement_value.distance
            self.engagements[id] = engagement_value.engagement
            self.mutual_gazes[id] = engagement_value.mutual_gaze
            

    def update_groups(self,group):
        self.g_time = group.header.stamp
        for id,conf in zip(group.members,group.confidences):
            if id in self.groups:
                self.groups[id] = group.group_id
                self.group_confidences[id] = conf
        

    '''
    UTIL
    '''

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--decision_maker", help="Which decision maker will be used",
                        type=str, default="heuristic")
    parser.add_argument("-r", "--robot_controller", help="Which robot controller will be used",
                        type=str, default="heuristic_ari_controller")
    parser.add_argument("--wait_time", help="Mean time to wait between decisions",
                        type=float, default=5)
    parser.add_argument("--wait_deviation", help="Standard deviation of wait times. If 0, will be always the mean time",
                        type=float, default=1)
    parser.add_argument("--robot", help="If true, will send commands to the robot",
                        type=str, default="True")
    parser.add_argument("--world_frame", help="World frame",
                        type=str, default="map")
    parser.add_argument("--z_offset", help="Offset to z axis when gazing to account for difference between eye and camera positions",
                        type=float, default=0.3)
    parser.add_argument("--reduced_action_space", help="If True, will limit the action space to a small subset",
                        type=str, default="True")
    parser.add_argument("--language", help="Language of the robot, can be 'english' or 'catalan'",
                        type=str, default="english")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("HRIDecide", anonymous=True)

    false_strings = ["False","false","f","F","0",False]
    robot = True
    reduced_action_space = True
    if args.robot in false_strings:
        robot = False
    if args.reduced_action_space in false_strings:
        reduced_action_space = False
    
    decision_node = DecisionNode(
        decision_maker = args.decision_maker,
        robot_controller = args.robot_controller,
        world_frame=args.world_frame,
        wait_time = args.wait_time,
        wait_deviation = args.wait_deviation,
        robot_command = robot,
        z_offset = args.z_offset,
        reduced_action_space=reduced_action_space,
        language=args.language,
        )
    decision_node.run()