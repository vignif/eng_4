import rospy

from engage.msg import Decision,EngagementLevel,MotionActivity

class SimpleDecisionMaker:
    def __init__(self,score_threshold=0,wait_time=5,stochastic=False):
        self.wait_time = rospy.Duration(wait_time)
        self.stochastic = stochastic
        self.score_threshold = score_threshold

    '''
    DECISION
    '''

    def decide(self,body_df,decision_vars):
        waiting = decision_vars["Waiting"]
        target,action = self.calculate_decision(body_df,waiting)
        return target,action

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