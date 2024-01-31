import rospy
import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge
import tf
import os
import rospkg

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped,Vector3,Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber

from opendr.engine.data import Image as OpenDRImage
from opendr.perception.pose_estimation import draw
from opendr.perception.pose_estimation import LightweightOpenPoseLearner

from engage.opendr_bridge import ROSBridge
from engage.pose_helper import HRIPoseManager

# This node in part adapts the OpenDR pose estimation node

class PoseEstimationNode:
    def __init__(
            self,
            rgb_image_topic,
            depth_image_topic,
            rgb_info_topic,
            depth_info_topic,
            model_path="",
            camera_frame="camera",
            world_frame="map",
            pose_image_topic=None,
            device="cuda",
            num_refinement_stages=2,
            use_stride=False,
            half_precision=False,
            rate=20,
            smoothing=True,
            smoothing_time=1,
            visualise=True
        ):
        # Rate
        self.rate = rospy.Rate(rate)


        # OpenDR pose estimator
        self.pose_estimator = LightweightOpenPoseLearner(device=device, num_refinement_stages=num_refinement_stages,
                                                         mobilenet_use_stride=use_stride,
                                                         half_precision=half_precision)
        
        print("Checking for model directory in {}".format(model_path))
        self.pose_estimator.download(path=model_path, verbose=True)
        self.pose_estimator.load(model_path+"openpose_default")

        self.opendr_bridge = ROSBridge()

        # Pose Manager
        self.visualise = visualise
        self.pose_manager = HRIPoseManager(
            visualise=visualise,
            smoothing=smoothing,
            window=smoothing_time*rate,
            camera_frame=camera_frame,
            world_frame=world_frame)

        # Subscribers
        self.rgb_image_sub = Subscriber(rgb_image_topic,Image)
        self.rgb_info_sub = Subscriber(rgb_info_topic,CameraInfo)
        self.depth_image_sub = Subscriber(depth_image_topic,Image)
        self.depth_info_sub = Subscriber(depth_info_topic,CameraInfo)

        time_slop = 1
        self.synch_sub = ApproximateTimeSynchronizer(
            [
                self.rgb_image_sub,
                self.rgb_info_sub,
                self.depth_image_sub,
                self.depth_info_sub
            ],
            10,
            time_slop,
        )
        self.synch_sub.registerCallback(self.camera_callback)
        self.im_time = None

        # Publishers
        self.opendr_pose_image_pub = None
        if pose_image_topic is not None:
            self.opendr_pose_image_pub = rospy.Publisher(pose_image_topic,Image,queue_size=1)

        # Transform Listener
        self.listener = tf.TransformListener()

        # Camera information for projections
        self.rgb_info = None
        self.depth_info = None
        self.camera_frame = camera_frame
        self.world_frame = world_frame

        # OpenCV Bridge
        self.cv_bridge = CvBridge()

        # Transforms
        self.cam_br = tf.TransformBroadcaster()

    def camera_callback(self,rgb_img,rgb_info,depth_img,depth_info):
        # Update time for synching
        self.im_time = rgb_info.header.stamp

        # Camera calibration
        if self.rgb_info is None:
            self.rgb_info = rgb_info
            self.depth_info = depth_info
            self.pose_manager.update_camera_model(rgb_info,depth_info)        

        # OpenCV
        rgb_image = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(rgb_img), cv2.COLOR_BGR2RGB)
        depth_image = self.cv_bridge.imgmsg_to_cv2(depth_img, "16UC1")

        # Get poses
        poses = self.opendr_pose_estimation(rgb_img)

        # Process new poses
        self.pose_manager.process_poses(poses,self.im_time,rgb_image,depth_image)

        # Visualise
        if self.visualise:
            for body in self.pose_manager.bodies:
                self.pose_manager.bodies[body].visualise_pose()

    def opendr_pose_estimation(self,rgb_img):
        
        # Convert sensor_msgs.msg.Image into OpenDR Image
        image = self.opendr_bridge.from_ros_image(rgb_img, encoding='bgr8')

        # Run pose estimation
        poses = self.pose_estimator.infer(image)

        # Publish pose image
        if self.opendr_pose_image_pub is not None:
            image = image.opencv()
            for pose in poses:
                draw(image, pose)
            image = self.opendr_bridge.to_ros_image(OpenDRImage(image), encoding='bgr8')
            image.header.stamp = self.im_time
            self.opendr_pose_image_pub.publish(image)
        # Return
        return poses


    def run(self):
        while not rospy.is_shutdown():
            # Listen for transform updates from the camera
            try:
                (trans,rot) = self.listener.lookupTransform(self.camera_frame,self.world_frame,rospy.Time(0))
                self.pose_manager.update_camera_transform(trans,rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                pass
            self.rate.sleep()


if __name__ == "__main__":
    default_camera = "camera"
    default_camera_frame = "camera_link"
    default_world_frame = "map"

    rospack = rospkg.RosPack()
    default_model_path = rospack.get_path('engage') + "/"

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--rgb_image_topic", help="Topic for rgb image",
                        type=str, default="/{}/color/image_raw".format(default_camera))
    parser.add_argument("-d", "--depth_image_topic", help="Topic for depth image",
                        type=str, default="/{}/depth/image_rect_raw".format(default_camera))
    parser.add_argument("-ii", "--rgb_info_topic", help="Topic for rgb camera info",
                        type=str, default="/{}/color/camera_info".format(default_camera))
    parser.add_argument("-di", "--depth_info_topic", help="Topic for depth camera info",
                        type=str, default="/{}/depth/camera_info".format(default_camera))
    parser.add_argument("-p", "--pose_image_topic", help="Topic for publishing annotated pose images",
                        type=str, default=None)
    parser.add_argument("--model_path", help="Path to the openpose model",
                        type=str, default=default_model_path)
    parser.add_argument("--camera_frame", help="Frame of the camera",
                        type=str, default=default_camera_frame)
    parser.add_argument("--world_frame", help="Frame of the world",
                        type=str, default=default_world_frame)
    parser.add_argument("--accelerate", help="Activates some acceleration features (e.g. reducing number of refinement steps)",
                        default=False)
    args = parser.parse_args(rospy.myargv()[1:])

    if args.accelerate:
        use_stride=True
        half_precision=True
        num_refinement_stages=0
    else:
        use_stride=False
        half_precision=False
        num_refinement_stages=2

    

    rospy.init_node("HRIPose", anonymous=True)

    print("Listening for images on {}".format(args.rgb_image_topic))
    
    pose_node = PoseEstimationNode(
        rgb_image_topic=args.rgb_image_topic,
        depth_image_topic=args.depth_image_topic,
        rgb_info_topic=args.rgb_info_topic,
        depth_info_topic=args.depth_info_topic,
        pose_image_topic=args.pose_image_topic,
        camera_frame=args.camera_frame,
        world_frame=args.world_frame,
        model_path=args.model_path,
        use_stride=use_stride,
        half_precision=half_precision,
        num_refinement_stages=num_refinement_stages
        )
    pose_node.run()
