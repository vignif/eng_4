import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
from itertools import combinations
import seaborn as sns

from engage.decision_maker.engage_state import EngageState

class EngageStateGrapher:
    single_value_graphs = [
        "Engagement Level with Robot",
        "Engagement Level Confidence",
        "Motion Activity",
        "Motion Activity Confidence",
    ]

    engage_value_graphs = [
        "Distance to Robot",
        "Engagement Score with Robot",
        "Mutual Gaze with Robot",
        "Pose Estimation Confidence",
    ]

    pairwise_graphs = [
        "Pairwise Distance",
        "Pairwise Engagement Score",
        "Pairwise Mutual Gaze",
    ]

    special_graphs = [
        "Decision",
        "Group Confidence",
        "Group Membership",
    ]

    graph_choices = sorted(single_value_graphs + engage_value_graphs + pairwise_graphs + special_graphs)

    def __init__(self):
        pass

    '''
    PLOT
    '''

    def plot(self,graph_name,bodies,pruned_bodies,log_bag,decisions,decision_times):
        if graph_name in self.special_graphs:
            # Graphs that have special features
            if graph_name == "Decision":
                fig,axes = self.plot_decision(graph_name,decisions,decision_times)
            elif graph_name == "Group Confidence":
                fig,axes = self.plot_group_confidence(graph_name,bodies,log_bag)
            elif graph_name == "Group Membership":
                fig,axes = self.plot_group_membership(graph_name,bodies,log_bag)
        elif graph_name in self.engage_value_graphs:
            # Graphs that contain one line for each body and use the EngagementValue message
            fig,axes = self.plot_engagement_value_msg(graph_name,pruned_bodies,log_bag)
        elif graph_name in self.pairwise_graphs:
            # Graphs for pairwise plots between bodies
            fig,axes = self.plot_pairwise(graph_name,pruned_bodies,log_bag)
        elif graph_name in self.single_value_graphs:
            # Graphs for single values per body
            fig,axes = self.plot_single_values(graph_name,pruned_bodies,log_bag)

        return fig,axes
    
    def plot_single_values(self,graph_name,bodies,log_bag):   
        if graph_name == "Engagement Level with Robot":
            subtopic = "engagement_status"
            label = "Engagement Level"
        elif graph_name == "Engagement Level Confidence":
            subtopic = "engagement_status"
            label = "Confidence"
        elif graph_name ==  "Motion Activity":
            subtopic = "activity"
            label = "Motion Activity"
        elif graph_name == "Motion Activity Confidence":
            subtopic = "activity"
            label = "Confidence"
        
        values = {body:[] for body in bodies}
        times = {body:[] for body in bodies}

        for body in bodies:
            for _,msg,_ in log_bag.read_messages(topics=["/humans/bodies/{}/{}".format(body,subtopic)]):
                if graph_name == "Engagement Level with Robot":
                    values[body].append(msg.level)
                elif graph_name == "Engagement Level Confidence":
                    values[body].append(msg.confidence)
                elif graph_name ==  "Motion Activity":
                    values[body].append(msg.activity)
                elif graph_name == "Motion Activity Confidence":
                    values[body].append(msg.confidence)
                times[body].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel(label)


        if graph_name == "Engagement Level with Robot":
            unique_decisions = EngageState.engagement_level_names
            plt.yticks(list(range(len(unique_decisions))),unique_decisions,rotation=45)
        elif graph_name == "Motion Activity":
            unique_decisions = EngageState.motion_names
            plt.yticks(list(range(len(unique_decisions))),unique_decisions,rotation=45)

        for body in bodies:
            plt.plot(times[body],values[body],label=body)
        plt.legend()

        return fig,[ax]

    
    def plot_pairwise(self,graph_name,bodies,log_bag):
        topic = "/humans/interactions/engagements"
        body_combos = list(combinations(bodies+["ROBOT"], 2))
        values = {"{}_{}".format(bc[0],bc[1]):[] for bc in body_combos}
        times = {"{}_{}".format(bc[0],bc[1]):[] for bc in body_combos}

        for _,msg,_ in log_bag.read_messages(topics=[topic]):
            body_a = msg.person_a
            body_b = msg.person_b if msg.person_b != "" else "ROBOT"
            combo = "{}_{}".format(body_a,body_b)
            if combo not in values:
                combo = "{}_{}".format(body_b,body_a)
                if combo not in values:
                    continue


            if graph_name == "Pairwise Distance":
                values[combo].append(msg.distance)
                label = "Distance"
            elif graph_name == "Pairwise Engagement Score":
                values[combo].append(msg.engagement)
                label = "Engagement Score"
            elif graph_name == "Pairwise Mutual Gaze":
                values[combo].append(msg.mutual_gaze)
                label = "Mutual Gaze"
            else:
                raise NotImplementedError

            times[combo].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel(label)

        for combo_pair in body_combos:
            combo = "{}_{}".format(combo_pair[0],combo_pair[1])
            plt.plot(times[combo],values[combo],label=combo)
        ax.legend()

        return fig,[ax]
                
    def plot_engagement_value_msg(self,graph_name,bodies,log_bag):
        topic = "/humans/interactions/engagements"
        values = {body:[] for body in bodies}
        times = {body:[] for body in bodies}
        for _,msg,_ in log_bag.read_messages(topics=[topic]):
            if msg.person_b == "" and msg.person_a in values: #ASSUMPTION: if robot is present, is always the second person
                # Involves the robot
                body = msg.person_a
                if graph_name == "Distance to Robot":
                    values[body].append(msg.distance)
                    label = "Distance"
                elif graph_name == "Engagement Score with Robot":
                    values[body].append(msg.engagement)
                    label = "Engagement Score"
                elif graph_name == "Mutual Gaze with Robot":
                    values[body].append(msg.mutual_gaze)
                    label = "Mutual Gaze"
                elif graph_name == "Pose Estimation Confidence":
                    values[body].append(msg.confidence_a)
                    label = "Confidence"
                else:
                    raise NotImplementedError

                times[body].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel(label)

        for body in bodies:
            plt.plot(times[body],values[body],label=body)
        ax.legend()

        return fig,[ax]
    
    def plot_group_confidence(self,graph_name,bodies,log_bag):
        topic = "/humans/interactions/groups"
        values = {body:[] for body in bodies+["ROBOT"]}
        times = {body:[] for body in bodies+["ROBOT"]}
        for _,msg,_ in log_bag.read_messages(topics=[topic]):
            for member,confidence in zip(msg.members,msg.confidences):
                values[member].append(confidence)
                times[member].append(msg.header.stamp.to_sec())
        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("Confidence")

        for body in bodies+["ROBOT"]:
            plt.plot(times[body],values[body],label=body)
        ax.legend()

        return fig,[ax]
    
    def plot_group_membership(self,graph_name,bodies,log_bag):
        topic = "/humans/interactions/groups"
        values = {}
        times = {}
        
        these_bodies = bodies+["ROBOT"]
        body_indices = {body:these_bodies.index(body) for body in these_bodies}

        for _,msg,_ in log_bag.read_messages(topics=[topic]):
            if len(msg.members)==0 or len(msg.members)==1:
                continue
            if msg.group_id not in values:
                values[msg.group_id] = {}
                times[msg.group_id] = {}
            for body in msg.members:
                if body in body_indices:
                    if body not in values[msg.group_id]:
                        values[msg.group_id][body] = []
                        times[msg.group_id][body] = []
                    values[msg.group_id][body].append(body_indices[body])
                    times[msg.group_id][body].append(msg.header.stamp.to_sec())

            for body in values[msg.group_id]:
                if body not in msg.members:
                    # Has left the group
                    values[msg.group_id][body].append(None)
                    times[msg.group_id][body].append(msg.header.stamp.to_sec())

        fig,ax = plt.subplots()
        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("Body")

        group_list = list(values.keys())
        plotted_groups = []

        plt.yticks(list(range(len(these_bodies))),these_bodies,rotation="horizontal")
        palette = sns.color_palette(None, len(group_list))
        for group in values:
            colour = palette[group_list.index(group)]
            for body in values[group]:
                plotted_groups.append(group)
                plt.plot(times[group][body],values[group][body],color=colour,label=group, linewidth=1)

        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        return fig,[ax]
                    
        

    def plot_decision(self,graph_name,decisions,decision_times):
        unique_decisions = sorted(list(set(decisions)))
        index_dict = {decision:unique_decisions.index(decision) for decision in unique_decisions}
        decision_enums = [index_dict[decision] for decision in decisions]
        fig,ax = plt.subplots()

        ax.set_title(graph_name)
        ax.set_xlabel("Time")
        ax.set_ylabel("Decision")
        plt.yticks(list(range(len(unique_decisions))),unique_decisions,rotation=45)

        plt.plot(decision_times,decision_enums)

        return fig,[ax]