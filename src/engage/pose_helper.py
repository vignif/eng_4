import rospy
import numpy as np
import math
from tsmoothie.smoother import LowessSmoother
import tf

from engage.utils import RandomID
from engage.marker_visualisation import MarkerMaker

from hri_msgs.msg import Skeleton2D, NormalizedPointOfInterest2D, IdsList
from sensor_msgs.msg import JointState
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker 

class HRIPoseBody:
    def __init__(self,id,time,visualise=False,smoothing=False,smoothing_window=None):
        self.id = id
        self.time = time
        self.visualise = visualise
        self.smoothing = smoothing
        self.smoothing_window = smoothing_window

        if visualise:
            self.marker_pub = rospy.Publisher("/markers",Marker,queue_size=100)

        if smoothing:
            self.smoother = LowessSmoother(smooth_fraction=0.4, iterations=1)
            
        self.pose = None
        self.pose_history = []

    def close(self):
        # To be called upon deletion
        self.marker_pub.unregister()

    def update(self,pose,time,rgb_image,rgb_model,depth_model,depth_image,cam_transform):
        self.pose = pose
        self.time = time

        # Update skeleton
        self.update_skeleton(rgb_image)

        # Update 3D points
        self.update_3D_pose(rgb_model,depth_model,depth_image,cam_transform)

    def update_skeleton(self,rgb_image):
        # Transform OpenDR pose into normalised 2D skeleton
        self.skeleton = Skeleton2D()
        self.skeleton.header.stamp = self.time

        self.skeleton.skeleton = [
            NormalizedPointOfInterest2D(
                kpt[0] / rgb_image.shape[1],
                kpt[1] / rgb_image.shape[0],
                self.pose.confidence
            ) 
            if (kpt[0] != -1 and kpt[1] != -1) else None for kpt in self.pose.data
        ]

    def update_3D_pose(self,rgb_model,depth_model,depth_image,inversed_transform):
        # Get coordinates in camera
        camera_coords = [self.point_2D_to_camera(kpt,rgb_model,depth_model,depth_image) for kpt in self.skeleton.skeleton]
        
        # Get world coords
        world_coords = []
        for cam_coord in camera_coords:
            if cam_coord is not None:
                world_coord = inversed_transform.dot(np.append(cam_coord,[1]))[0:3]
            else:
                world_coord = None
            world_coords.append(world_coord)
        
        marker = MarkerMaker.make_marker_sphere(world_coords[1],self.id,0,colour=[1,0,0])
        if marker is not None:
            self.marker_pub.publish(marker)




    def point_2D_to_camera(self,point,rgb_model,depth_model,depth_image):
        if point is None:
            return None
        
        px = min(math.floor(point.x * depth_image.shape[1]), depth_image.shape[1] - 1)
        py = min(math.floor(point.y * depth_image.shape[0]), depth_image.shape[0] - 1)

        x_d = int(((px - rgb_model.cx())
               * depth_model.fx()
               / rgb_model.fx())
              + depth_model.cx())
        y_d = int(((py - rgb_model.cy())
                * depth_model.fy()
                / rgb_model.fy())
                + depth_model.cy())
        z = depth_image[y_d][x_d]/1000

        x = (x_d - depth_model.cx())*z/depth_model.fx()
        y = (y_d - depth_model.cy())*z/depth_model.fy()

        return np.array([-z,x,-y])

    

class HRIPoseManager:
    def __init__(self,body_timeout=0.1,visualise=True,smoothing=True,smoothing_window=40):
        # Parameters
        self.body_timeout = rospy.Duration(body_timeout)
        self.visualise = visualise
        self.smoothing = smoothing
        self.smoothing_window = smoothing_window
        # Map from pose id to body id
        self.body_ids = {}
        # Dict of all bodies currently tracked
        self.bodies = {}

    def update_camera_model(self,rgb_info,depth_info):
        self.depth_model = PinholeCameraModel()
        self.rgb_model = PinholeCameraModel()

        self.depth_model.fromCameraInfo(depth_info)
        self.rgb_model.fromCameraInfo(rgb_info)

    def update_camera_transform(self,cam_transform):
        trans = cam_transform.transform.translation
        quat = cam_transform.transform.rotation
        transform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix([trans.x,trans.y,trans.z]), 
            tf.transformations.quaternion_matrix([quat.x,quat.y,quat.z,quat.w])
        )
        self.inversed_transform = tf.transformations.inverse_matrix(transform)

    def process_poses(self,poses,time,rgb_image,depth_image):
        bodies_in_pose = []

        # Start by updating the list of bodies
        for pose in poses:
            if pose.id not in self.body_ids:
                # New body
                self.body_ids[pose.id] = RandomID.random_id(pose.id)
                self.bodies[self.body_ids[pose.id]] = HRIPoseBody(
                    self.body_ids[pose.id],
                    time,
                    visualise=self.visualise,
                    smoothing=self.smoothing,
                    smoothing_window=self.smoothing_window
                )
            self.bodies[self.body_ids[pose.id]].update(
                pose,
                time,
                rgb_image,
                self.rgb_model,
                self.depth_model,
                depth_image,
                self.inversed_transform
            )
            bodies_in_pose.append(self.body_ids[pose.id])
        
        # Now, remove any bodies that have timed out
        for body in list(self.bodies.keys()):
            if body not in bodies_in_pose and abs(time - self.bodies[body].time) > self.body_timeout:
                self.bodies[body].close()
                del self.bodies[body]

