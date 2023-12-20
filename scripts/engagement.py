import rospy
import argparse
import numpy as np
import tf

from engage.engage_helper import HRIEngagementManager

from hri_msgs.msg import IdsList

class EngagementNode:
    def __init__(
            self,
            engagement_threshold=0.55,
            max_mutual_gaze_angle=np.pi,
            window_time=1,
            ignore_z=True,
            camera_frame="camera",
            world_frame="world",
            rate=20,
        ):
        # Rate
        self.rate = rospy.Rate(rate)
        self.time = None

        # Parameters
        self.engagement_threshold = engagement_threshold
        self.max_mutual_gaze_angle = max_mutual_gaze_angle
        self.camera_frame = camera_frame
        self.world_frame = world_frame

        # Engagement manager
        self.engagement_manager = HRIEngagementManager(
            engagement_threshold=engagement_threshold,
            max_mutual_gaze_angle=max_mutual_gaze_angle,
            window_size=window_time*rate,
            ignore_z=ignore_z,
            camera_frame=camera_frame,
            world_frame=world_frame
            )

        # Subscriber
        self.body_subscriber = rospy.Subscriber("/humans/bodies/tracked",IdsList,self.body_callback)

        # Transform Listener
        self.listener = tf.TransformListener()

    def body_callback(self,ids_list):
        # Unpack IdsList
        self.time = ids_list.header.stamp
        body_list = ids_list.ids
        
        # Manage bodies
        self.engagement_manager.manage_bodies(self.time,body_list)

        # Calculate engagements
        distances,mutual_gazes,engagements = self.engagement_manager.calculate_engagement()

        # Calculate activities
        self.engagement_manager.calculate_motion()

        # Calculate groups
        group_confidences = self.engagement_manager.calculate_groups(engagements)

        # Publishing
        self.engagement_manager.publish_engagements(distances,mutual_gazes,engagements)
        self.engagement_manager.publish_motion()
        self.engagement_manager.publish_groups(group_confidences)

        # Debugging
        for body in self.engagement_manager.bodies:
            print("{}'s mutual gaze: {}".format(body,self.engagement_manager.bodies[body].mutual_gaze_robot))

        

    def run(self):
        while not rospy.is_shutdown():
            
            # Listen for transform updates from the camera
            try:
                (trans,rot) = self.listener.lookupTransform(self.camera_frame,self.world_frame,rospy.Time(0))
                self.engagement_manager.update_robot_position(trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                pass

            self.rate.sleep()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--engagement_threshold", help="Threshold over which someone is considered engaged",
                        type=float, default=0.55)
    parser.add_argument("--max_angle", help="Angle over which mutual gaze is 0",
                        type=float, default=np.pi/2)
    parser.add_argument("--camera_frame", help="Camera Frame",
                        type=str, default="camera")
    parser.add_argument("--world_frame", help="World Frame",
                        type=str, default="world")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("HRIEngage", anonymous=True)
    
    engage_node = EngagementNode(
        engagement_threshold=args.engagement_threshold,
        max_mutual_gaze_angle=args.max_angle,
        camera_frame=args.camera_frame,
        world_frame=args.world_frame
        )
    engage_node.run()
