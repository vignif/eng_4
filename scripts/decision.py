import numpy as np
import rospy
import networkx as nx
import pandas as pd

# Hide that one annoying pandas warning
pd.options.mode.chained_assignment = None 

from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD

from engage.msg import EngagementValue,Group,EngagementLevel,MotionActivity,Decision,PoseArrayUncertain
from hri_msgs.msg import IdsList
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import PointStamped
from engage.bn_utils import bn_predict
from engage.decision_maker import DecisionMaker,ACTION_NAMES
from play_motion_msgs.msg import PlayMotionActionGoal
from pal_interaction_msgs.msg import TtsActionGoal
from engage.pose_helper import HRIPoseBody

CSV_DIR = "~/catkin_ws/src/hriri/logging/decision_csvs/"

RATE = 1
DECISION_LOCK = 5 # If a decision other than "nothing" is taken, wait this long
TIME_SLOP = 0.1

class Body:
    def __init__(self,id) -> None:
        self.id = id
        body_topic = "/humans/bodies/{}/".format(self.id)

        # Subscribers
        self.engagement_level_subscriber = Subscriber(body_topic+"engagement_status",EngagementLevel)
        self.activity_subscriber = Subscriber(body_topic+"activity",MotionActivity)

        self.position_subscriber = rospy.Subscriber("/humans/bodies/{}/poses".format(id),PoseArrayUncertain,self.update_position)

        time_slop = TIME_SLOP
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


class DecisionMakingNode:
    def __init__(self,saving=True,exp_name=None) -> None:
        self.rate = rospy.Rate(RATE)
        self.lock = False
        self.unlock_time = None
        self.waiting = False

        self.decision_table = None
        self.saving = saving

        if exp_name is None:
            exp_name = "UntitledExperiment"
        self.exp_name = exp_name

        # Subscribers
        self.body_subscriber = rospy.Subscriber("/humans/bodies/tracked",IdsList,self.manage_bodies)
        self.engagement_value_subscriber = rospy.Subscriber("/humans/interactions/engagements",EngagementValue,self.update_engagements)
        self.group_subscriber = rospy.Subscriber("/humans/interactions/groups",Group,self.update_groups)

        # Publishers
        self.motion_action_publisher = rospy.Publisher("/play_motion/goal",PlayMotionActionGoal,queue_size=1)
        self.gaze_action_publisher = rospy.Publisher("/look_at",PointStamped,queue_size=1)
        self.tts_publisher = rospy.Publisher("/tts/goal",TtsActionGoal,queue_size=1)

        self.body_time = None
        self.dec_time = None
        self.decision_time = None
        self.bodies = {}
        self.groups = {}
        self.group_confidences = {}
        self.distances = {}
        self.engagements = {}
        self.mutual_gazes = {}
        self.pose_confidences = {}

        self.decision_maker = DecisionMaker()



    def __del__(self):
        self.close()

    def close(self):
        self.body_subscriber.unregister()

        self.engagement_value_subscriber.unregister()
        self.group_subscriber.unregister()


    '''
    
    Main Callback
    
    '''

    def manage_bodies(self,tracked_msg):

        if self.lock:
            # Skip
            return
        
        if self.unlock_time is not None and rospy.Time.now() < self.unlock_time:
            self.waiting = True
        else:
            self.waiting = False

        self.lock = True
        self.body_time = tracked_msg.header.stamp
        self.dec_time = rospy.Time.now()
        body_list = tracked_msg.ids

        tracked_ids = set(body_list)
        managed_ids = set(self.bodies.keys())

        # Add new ids
        ids_to_add = list(tracked_ids - managed_ids)
        for id in ids_to_add:
            self.bodies[id] = Body(id)
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

        # Make a decision
        body_df = self.compile_body_df()
        body_df,target,action = self.decision_maker.decide(body_df,self.waiting)

        # Compile information
        if self.saving:
            if self.decision_table is None:
                self.decision_table = body_df
            else:
                self.decision_table = pd.concat([self.decision_table,body_df],ignore_index=True)

        # Timer
        if action != Decision.NOTHING and action != Decision.WAIT:
            self.unlock_time = rospy.Time.now() + rospy.Duration(DECISION_LOCK)
            print("Executing action {} with target {}".format(ACTION_NAMES[action],target))
        self.execute_action(action,target)

        # Unlock
        self.lock = False

    '''
    Robot actions
    '''

    def execute_action(self,action,target):
        # TODO: Fix all of this
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
            target_body = self.bodies[target]
            if target_body.position is not None:
                target_pos = PointStamped()
                target_pos.header.frame_id = "map"
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

        

    '''
    
    Subscribers
    
    '''

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
    
    Body DF
    
    '''
    def compile_body_df(self):
        headers = ["Body Time","Dec Time","Body","Group","G","GC","A","AC","EL","ELC","MG","D","EV","PC","GWR"]
        table = []
        for id in self.bodies:
            if self.bodies[id].engagement_level is None or self.bodies[id].activity is None or self.distances[id] is None or self.groups[id] is None:
                continue
            body_row = [
                self.body_time,
                self.dec_time,
                id,
                self.groups[id],
                self.in_group(id),
                self.float_bucket(self.group_confidences[id]),
                self.bodies[id].activity,
                self.float_bucket(self.bodies[id].activity_confidence),
                self.bodies[id].engagement_level,
                self.float_bucket(self.bodies[id].engagement_level_confidence),
                self.float_bucket(self.mutual_gazes[id]),
                self.distance_bucket(self.distances[id]),
                self.float_bucket(self.engagements[id]),
                self.float_bucket(self.pose_confidences[id]),
                self.groups[id] == self.groups["ROBOT"],
            ]
            table.append(body_row)
        # ROBOT
        if self.group_confidences["ROBOT"] is not None:
            table.append([
                self.body_time,
                self.dec_time,
                "ROBOT",
                self.groups["ROBOT"],
                self.in_group("ROBOT"),
                self.float_bucket(self.group_confidences["ROBOT"]),
                0,
                3,
                0,
                3,
                0,
                0,
                0,
                1,
                True
                ])
        return pd.DataFrame(table,columns=headers)

    def in_group(self,id):
        # True if id's group > 1 person
        same_group = [k for k,v in self.groups.items() if v == self.groups[id]]
        return len(same_group)>1

    def float_bucket(self,value):
        if value < 0.25:
            return 0
        elif value < 0.5:
            return 1
        elif value < 0.75:
            return 2
        else:
            return 3

    def distance_bucket(self,distance):
        if distance <= 3:
            discretised_distance = round(distance * 2) / 2
        else:
            discretised_distance = round(distance)

        if discretised_distance == 0.0:
            discretised_distance = 0.1
        elif discretised_distance > 8:
            discretised_distance = 9

        return discretised_distance

    '''
    
    Save Data
    
    '''

    def save_data(self,csv_file=None):
        if csv_file is None:
            csv_file = self.exp_name + ".csv"
        if self.saving:
            # Start by removing duplicates
            ddf = self.decision_table.copy()
            ddf = ddf.drop_duplicates(subset=["Dec Time","Body","Decision"],keep="last")
            # Get bodies and times
            bodies = ddf["Body"].unique()
            times = ddf["Dec Time"].unique()

            # Create data
            data = pd.DataFrame(times,columns=["Dec Time"])
            rn_cols = ["Group","G","GC","TG","EL","ELC","TEL","A","AC","TA","MG","D","EV","PC","GWR","Score","Decision","Target","Waiting","Body Time"]
            for body in bodies:
                bdf = ddf.loc[ddf["Body"]==body].copy()
                drop_cols = ["Body"]
                if body == "ROBOT":
                    drop_cols += ["EL","ELC","TEL","A","TA","AC","MG","D","EV","PC","GWR","Score"]
                bdf = bdf.drop(columns=drop_cols)
                data = data.merge(bdf,on=["Dec Time"],how='left')
                mapping = {k: body+"_"+k for k in rn_cols}
                data.rename(columns=mapping,inplace=True)

            # Combine columns
            wait_cols = [b+"_Waiting" for b in bodies]
            data["Waiting"] = data[wait_cols].any(axis=1)

            body_cols = [b+"_Body Time" for b in bodies]
            data["Body Time"] = data[body_cols].fillna(method='bfill', axis=1).iloc[:, 0]

            dec_cols = [b+"_Decision" for b in bodies]
            data["Decision"] = data[dec_cols].fillna(method='bfill', axis=1).iloc[:, 0]

            tar_cols = [b+"_Target" for b in bodies]
            data["Target"] = data[tar_cols].fillna(method='bfill', axis=1).iloc[:, 0]

            data = data.drop(columns=wait_cols+body_cols+dec_cols+tar_cols)

            print(data)
            data.to_csv(CSV_DIR+csv_file,index=False)
            
            
        


    '''
    
    Basic Stuff
    
    '''


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        self.save_data()

if __name__ == "__main__":
    rospy.init_node("HRIDecision", anonymous=True)
    try:
        exp_name = rospy.get_param("exp_name")
    except:
        exp_name = "Test"
    dmn = DecisionMakingNode(saving=False,exp_name=exp_name)
    dmn.run()

