import pandas as pd
import rospy
import numpy as np

from engage.msg import Decision,EngagementLevel,MotionActivity

class SimpleDecisionMaker:
    def __init__(self,score_threshold=0,wait_time=5,stochastic=False):
        self.wait_time = rospy.Duration(wait_time)
        self.stochastic = stochastic
        self.score_threshold = score_threshold

        self.last_decision_time = None

    '''
    DECISION
    '''

    def decide(self,decision_node):
        # Copy relevant stuff from decision_node
        decision_node = self.copy_decision_node(decision_node)

        if self.last_decision_time is None or decision_node.dec_time - self.last_decision_time > self.wait_time:
            waiting = False
        else:
            waiting = True

        body_df = self.compile_body_df(decision_node)
        target,action = self.calculate_decision(body_df,waiting)

        if action != Decision.NOTHING and action != Decision.WAIT:
            self.last_decision_time = decision_node.dec_time

        return target,action,decision_node.dec_time

    def calculate_decision(self,body_df,waiting):
        target = None
        action = None
        
        robot_row_df = body_df.loc[body_df['Body'] == "ROBOT"]
        people_df = body_df.loc[body_df['Body'] != "ROBOT"]
        # TODO: I am putting the scoring here so that the score is always available even when it isn't used. For performance, this should probably be in the else statement
        people_df["Score"] = people_df.apply(self.score_bodies,axis=1)
        body_df["Score"] = people_df["Score"]
        group_member_df = people_df.loc[body_df["GWR"]]
        if waiting:
            # Waiting for an action to execute
            action = Decision.WAIT
        elif people_df.empty:
            # Nobody to interact with
            action = Decision.NOTHING
        elif not robot_row_df.empty and robot_row_df.iloc[0]["G"]==1 and not group_member_df.empty:
            # Robot in a group
            els = group_member_df["EL"].to_numpy()
            if (els == EngagementLevel.ENGAGED).all():
                # Everyone engaged, maintain engagement with random target
                if self.stochastic:
                    target = group_member_df.sample(n=1)["Body"].iloc[0]
                else:
                    target = group_member_df["Body"].iloc[0]
                action = Decision.MAINTAIN
            else:
                # Someone not engaged, recapture
                disengaged_group_member_df = group_member_df.loc[(group_member_df["EL"] != EngagementLevel.ENGAGED)]
                if self.stochastic:
                    target = disengaged_group_member_df.sample(n=1)["Body"].iloc[0]
                else:
                    target = disengaged_group_member_df["Body"].iloc[0]
                action = Decision.RECAPTURE
        else:
            # Robot not in a group, must try grab someone's attention
            potential_targets = people_df[people_df['Score'] == people_df['Score'].max()]
            print(people_df['Score'])
            valid_targets = potential_targets.loc[potential_targets["Score"]>self.score_threshold]
            if valid_targets.empty:
                action = Decision.ELICIT_GENERAL
            else:
                if self.stochastic:
                    target = valid_targets.sample(n=1)["Body"].iloc[0]
                else:
                    target = valid_targets["Body"].iloc[0]
                action = Decision.ELICIT_TARGET

        return target,action
    
    def score_bodies(self,df):
        return self.score_body(df["G"],df["MG"],df["D"],df["EL"],df["A"],df["GC"],df["AC"],df["ELC"],df["PC"])
    
    def score_body(self,
                   G,
                   MG,
                   D,
                   EL,
                   A,
                   GC,
                   AC,
                   ELC,
                   PC,
                   gaze_thresh=0.75,
                   close_dist=1,
                   mid_dist=3,
                   max_float_bucket=3,
                   group_weight=1,
                   engagement_weight=1,
                   activity_weight=1,
                   distance_weight=1):
        '''
        
        Give every individual a score
                + if in a group and mutual gaze is high
                + for engagement value
                + if walking towards
                - if in a group and mutual gaze is low
                - if walking away and mutual gaze is low
        
        '''

        score = 0
        if G==1:
            # Person in a group...
            if (MG/max_float_bucket)>gaze_thresh:
                # ...and looking at robot
                score += group_weight*(GC/max_float_bucket)*(PC/max_float_bucket)
            else:
                # ...and looking away
                score -= group_weight*(GC/max_float_bucket)*(PC/max_float_bucket)
        if (MG/max_float_bucket)>gaze_thresh:
            score += engagement_weight*(PC/max_float_bucket)
        else:
            score -= engagement_weight*(PC/max_float_bucket)
        
        
        if EL==EngagementLevel.ENGAGED or EL==EngagementLevel.ENGAGING:
            # Person is engaged or engaging
            score += engagement_weight*(ELC/max_float_bucket)

        
        if A==MotionActivity.WALKING_TOWARDS:
            # person is walking towards the robot
            score += activity_weight*(AC/max_float_bucket)
        elif A==MotionActivity.WALKING_AWAY:
            # person is walking away from the robot
            score -= activity_weight*(AC/max_float_bucket)

        # Score based on distance
        if D < mid_dist:
            score += distance_weight*(1-(D/mid_dist))*(PC/max_float_bucket)
            if D < close_dist:
                score += distance_weight*(1-(D/close_dist))*(PC/max_float_bucket)
        else:
            score -= distance_weight*(PC/max_float_bucket)
        
        return score

    '''
    DATAFRAME
    '''
    def copy_decision_node(self,decision_node):
        # TODO - make a copy of the decision node and the bodies so that all variables are frozen for the decision
        return decision_node

    def compile_body_df(self,decision_node):
        headers = ["Body Time","Dec Time","Body","Group","G","GC","A","AC","EL","ELC","MG","D","EV","PC","GWR"]
        table = []
        for id in decision_node.bodies:
            if decision_node.bodies[id].engagement_level is None or decision_node.bodies[id].activity is None or decision_node.distances[id] is None or decision_node.groups[id] is None:
                continue
            body_row = [
                decision_node.body_time,
                decision_node.dec_time,
                id,
                decision_node.groups[id],
                self.in_group(id,decision_node),
                self.float_bucket(decision_node.group_confidences[id]),
                decision_node.bodies[id].activity,
                self.float_bucket(decision_node.bodies[id].activity_confidence),
                decision_node.bodies[id].engagement_level,
                self.float_bucket(decision_node.bodies[id].engagement_level_confidence),
                self.float_bucket(decision_node.mutual_gazes[id]),
                self.distance_bucket(decision_node.distances[id]),
                self.float_bucket(decision_node.engagements[id]),
                self.float_bucket(decision_node.pose_confidences[id]),
                decision_node.groups[id] == decision_node.groups["ROBOT"],
            ]
            table.append(body_row)
        # ROBOT
        if decision_node.group_confidences["ROBOT"] is not None:
            table.append([
                decision_node.body_time,
                decision_node.dec_time,
                "ROBOT",
                decision_node.groups["ROBOT"],
                self.in_group("ROBOT",decision_node),
                self.float_bucket(decision_node.group_confidences["ROBOT"]),
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

    def in_group(self,id,decision_node):
        # True if id's group > 1 person
        same_group = [k for k,v in decision_node.groups.items() if v == decision_node.groups[id]]
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