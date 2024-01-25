import rospy
import argparse

from message_filters import ApproximateTimeSynchronizer, Subscriber
from engage.msg import EngagementValue,Group,EngagementLevel,MotionActivity,Decision,PoseArrayUncertain,StateDecision
from hri_msgs.msg import IdsList

from engage.decision_maker.simple_decision_maker import SimpleDecisionMaker
from engage.pose_helper import HRIPoseBody
from engage.robot_controller import SimpleARIController
from engage.decision_helper import DecisionState

class DecisionBody:
    def __init__(self,id):
        self.id = id

        body_topic = "/humans/bodies/{}/".format(self.id)

        # Subscribers
        self.engagement_level_subscriber = Subscriber(body_topic+"engagement_status",EngagementLevel)
        self.activity_subscriber = Subscriber(body_topic+"activity",MotionActivity)

        self.position_subscriber = rospy.Subscriber("/humans/bodies/{}/poses".format(id),PoseArrayUncertain,self.update_position)

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

    def __del__(self):
        self.close()
    
    def close(self):
        self.engagement_level_subscriber.sub.unregister()
        self.activity_subscriber.sub.unregister()
        self.position_subscriber.unregister()


    def update_body(self,engagement_level,activity):
        self.engagement_level = engagement_level.level
        self.engagement_level_confidence = engagement_level.confidence
        self.activity = activity.activity
        self.activity_confidence = activity.confidence

    def update_position(self,pose):
        if pose.poses[HRIPoseBody.joints["nose"]] is not None:
            self.position = pose.poses[HRIPoseBody.joints["nose"]]

class DecisionNode:
    decision_makers = {
        "simple_decision_maker":SimpleDecisionMaker,
    }

    robot_controllers = {
        "simple__ari_controller":SimpleARIController,
    }

    def __init__(
            self,
            decision_maker="simple_decision_maker",
            robot_controller="simple__ari_controller",
            rate=20,
            robot_command=True,
            wait_time=5
    ):
        # Rate
        self.rate = rospy.Rate(rate)
        self.last_decision_time = None
        self.lock = False
        self.wait_time = rospy.Duration(wait_time)

        # Decision-Maker
        self.dm = self.decision_makers[decision_maker]()

        # Robot Controller
        self.robot_command = robot_command
        if robot_command:
            self.robot_controller = self.robot_controllers[robot_controller]()

        # Subscribers
        self.body_subscriber = rospy.Subscriber("/humans/bodies/tracked",IdsList,self.manage_bodies)
        self.engagement_value_subscriber = rospy.Subscriber("/humans/interactions/engagements",EngagementValue,self.update_engagements)
        self.group_subscriber = rospy.Subscriber("/humans/interactions/groups",Group,self.update_groups)

        # Publishers
        self.decision_publisher = rospy.Publisher("/hri_engage/decisions",Decision,queue_size=1)
        self.decision_state_publisher = rospy.Publisher("hri_engage/decision_states",StateDecision,queue_size=1)

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
        self.state = DecisionState(self.body_time,
                                   self.bodies,
                                   self.groups,
                                   self.group_confidences,
                                   self.distances,
                                   self.engagements,
                                   self.mutual_gazes,
                                   self.pose_confidences,
                                   self.is_waiting()
                                   )
        target,action = self.dm.decide(self.state)

        if action != Decision.NOTHING and action != Decision.WAIT:
            self.last_decision_time = self.body_time

        # Publish decision
        self.publish_decision(target,action,self.body_time)

        # Publish state
        self.publish_decision_state(self.state,action,target)

        # Control robot
        if self.robot_command:
            self.robot_controller.execute_command(target,action,self.bodies,self.body_time)

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
    STATE
    '''
    def is_waiting(self):
        if self.last_decision_time is None or self.body_time - self.last_decision_time > self.wait_time:
            return False
        else:
            return True

    '''
    PUBLISHING
    '''

    def publish_decision(self,target,action,time):
        decision = Decision()
        decision.header.stamp = time
        decision.decision = action
        decision.target = target if target is not None else ""
        self.decision_publisher.publish(decision)

    

    def publish_decision_state(self,state:DecisionState,action:int,target:str):
        self.decision_state_publisher.publish(state.message(action,target))
        

    '''
    UTIL
    '''

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--decision_maker", help="Which decision maker will be used",
                        type=str, default="simple_decision_maker")
    parser.add_argument("--wait_time", help="Fixed time to wait between decisions",
                        type=float, default=5)
    parser.add_argument("--robot", help="If true, will send commands to the robot",
                        type=str, default="True")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("HRIDecide", anonymous=True)

    robot = True
    if args.robot in ["False","false","f","F","0"]:
        robot = False
    
    decision_node = DecisionNode(
        decision_maker = args.decision_maker,
        wait_time = args.wait_time,
        robot_command = robot,
        )
    decision_node.run()