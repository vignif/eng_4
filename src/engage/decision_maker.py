import pandas as pd
import numpy as np
from pgmpy.models import BayesianNetwork
from pgmpy.factors.discrete import TabularCPD

from engage.msg import EngagementValue,Group,EngagementLevel,Activity,Decision
from engage.bn_utils import bn_predict

ACTION_NAMES = ["NOTHING","WAIT","MAINTAIN","RECAPTURE","ELICIT_TARGET","ELICIT_GENERAL"]

class DecisionMaker:
    def __init__(self):
        self.setup_bns()


    '''
    
    Setup
    
    '''

    def setup_bns(self):
        # Set up all the BNs
        self.confidence_bn = self.setup_confidence_bn()

    def setup_confidence_bn(self):
        model = BayesianNetwork([("TG","G"),("GC","G"),("TA","A"),("AC","A"),("TEL","EL"),("ELC","EL")])
        cpd_tg = TabularCPD("TG",2,[[0.6],[0.4]])
        cpd_gc = TabularCPD("GC",4,[[0.05],[0.1],[0.25],[0.6]])
        cpd_g = TabularCPD(
            "G",
            2,
            [
                [0.4,0.5,0.75,1,0.6,0.5,0.25,0],
                [0.6,0.5,0.25,0,0.4,0.5,0.75,1]
            ],
            evidence = ["TG","GC"],
            evidence_card = [2,4]
        )
        model.add_cpds(cpd_tg,cpd_gc,cpd_g)

        cpd_ta = TabularCPD("TA",4,[[0.25],[0.25],[0.25],[0.25]])
        cpd_ac = TabularCPD("AC",4,[[0.05],[0.1],[0.25],[0.6]])
        cpd_a = TabularCPD(
            "A",
            4,
            [
                [0.31,0.4,0.61,1,0.2,0.04,0.05,0,0.20,0.15,0.08,0,0.20,0.1,0.05,0],
                [0.23,0.2,0.13,0,0.3,0.60,0.70,1,0.15,0.10,0.04,0,0.25,0.2,0.10,0],
                [0.23,0.2,0.13,0,0.2,0.14,0.10,0,0.30,0.50,0.70,1,0.25,0.2,0.10,0],
                [0.23,0.2,0.13,0,0.3,0.22,0.15,0,0.35,0.25,0.18,0,0.30,0.5,0.75,1]
            ],
            evidence = ["TA","AC"],
            evidence_card = [4,4]
        )
        model.add_cpds(cpd_ta,cpd_ac,cpd_a)


        cpd_tel = TabularCPD("TEL",5,[[0.12],[0.22],[0.22],[0.22],[0.22]])
        cpd_elc = TabularCPD("ELC",4,[[0.05],[0.1],[0.25],[0.6]])
        cpd_el = TabularCPD(
            "EL",
            5,
            [
                [1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0.30,0.50,0.70,1,0.20,0.15,0.05,0,0.20,0.07,0.05,0,0.26,0.20,0.10,0],
                [0,0,0,0,0.25,0.18,0.10,0,0.30,0.50,0.70,1,0.25,0.22,0.13,0,0.26,0.20,0.10,0],
                [0,0,0,0,0.20,0.07,0.05,0,0.30,0.20,0.15,0,0.30,0.50,0.70,1,0.18,0.10,0.10,0],
                [0,0,0,0,0.25,0.25,0.15,0,0.20,0.15,0.10,0,0.25,0.21,0.12,0,0.30,0.50,0.70,1],
            ],
            evidence = ["TEL","ELC"],
            evidence_card = [5,4]
        )
        model.add_cpds(cpd_tel,cpd_elc,cpd_el)


        return model


    '''
    
    Body DF
    
    '''

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
        if distance < 1:
            return 0
        elif distance < 3:
            return 1
        elif distance < 5:
            return 2
        else:
            return 3


    '''
    
    Decision Making

    '''

    def decide(self,body_df,waiting,score_threshold=0,stochastic=False):
        prediction = bn_predict(self.confidence_bn,body_df[["G","GC","A","AC","EL","ELC"]],["TG","TA","TEL"])
        body_df = pd.concat([body_df,prediction],axis=1)
        body_df["Waiting"] = waiting

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
        elif not robot_row_df.empty and robot_row_df.iloc[0]["TG"]==1 and not group_member_df.empty:
            # Robot in a group
            els = group_member_df["TEL"].to_numpy()
            if (els == EngagementLevel.ENGAGED).all():
                # Everyone engaged, maintain engagement with random target
                if stochastic:
                    target = group_member_df.sample(n=1)["Body"].iloc[0]
                else:
                    target = group_member_df["Body"].iloc[0]
                action = Decision.MAINTAIN
            else:
                # Someone not engaged, recapture
                disengaged_group_member_df = group_member_df.loc[(group_member_df["TEL"] != EngagementLevel.ENGAGED)]
                if stochastic:
                    target = disengaged_group_member_df.sample(n=1)["Body"].iloc[0]
                else:
                    target = disengaged_group_member_df["Body"].iloc[0]
                action = Decision.RECAPTURE
        else:
            # Robot not in a group, must try grab someone's attention
            potential_targets = people_df[people_df['Score'] == people_df['Score'].max()]
            valid_targets = potential_targets.loc[potential_targets["Score"]>score_threshold]
            if valid_targets.empty:
                action = Decision.ELICIT_GENERAL
            else:
                if stochastic:
                    target = valid_targets.sample(n=1)["Body"].iloc[0]
                else:
                    target = valid_targets["Body"].iloc[0]
                action = Decision.ELICIT_TARGET

        body_df["Decision"] = action
        body_df["Target"] = np.nan if target is None else target

        return body_df,target,action
    
    def score_bodies(self,df):
        return self.score_body(df["TG"],df["MG"],df["D"],df["TEL"],df["TA"],df["GC"],df["AC"],df["ELC"],df["PC"])
    
    def score_body(self,
                   TG,
                   MG,
                   D,
                   TEL,
                   TA,
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
        if TG==1:
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
        
        
        if TEL==EngagementLevel.ENGAGED or TEL==EngagementLevel.ENGAGING:
            # Person is engaged or engaging
            score += engagement_weight*(ELC/max_float_bucket)
        
        if TA==Activity.WALKING_TOWARDS:
            # person is walking towards the robot
            score += activity_weight*(AC/max_float_bucket)
        elif TA==Activity.WALKING_AWAY:
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