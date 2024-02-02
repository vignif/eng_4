# Engage

A ros noetic package for planning and executing robot-human elicitations in multi-person unstructured environments. See our [late breaking report at HRI 2024](https://www.iri.upc.edu/groups/perception/#TRAIL_XHRI) to learn more.

# Installation

This package requires Ubuntu 20.04 and ROS noetic. It also depends on the pyhri and hri_msgs packages for ROS4HRI (installation instructions [here](http://wiki.ros.org/hri/Tutorials/Installation%20of%20a%20ROS4HRI%20environment)), and the [play_motion_msgs](https://github.com/pal-robotics/play_motion) and [pal_interaction_msgs](https://github.com/pal-robotics/pal_msgs) packages from Pal robotics. For pose estimation, this package relies on OpenDR, available [here](https://github.com/opendr-eu/opendr/blob/master/docs/reference/installation.md). Note that this implementation has only been tested with the GPU version of OpenDR, using pytorch 1.13.1+cu117.

For python packages required, see the [requirements.txt](https://github.com/tamlinlove/engage/blob/main/requirements.txt) file. Use of a virtual environment, such as [rosvenv](https://github.com/ARoefer/rosvenv), is highly recommended.

Hardware-wise, this package requires an RGB and depth stream from a camera, such as a realsense D435i.

# Usage

In this package there are four main scripts: one for pose estimation, one for calculating high-level features, one for calculating decisions, and one for visualising rosbags and explaining decisions post-hoc.

## Pose Estimation

If all you want to do is pose estimation, you can run the following:

`roslaunch engage pose.launch`

which takes the following arguments:

- cam_name - *default: camera*, the name of the camera topic
- rgb_img - *default: /$(arg cam_name)/color/image_raw*, the topic for the RGB image stream
- dep_img - *default: /$(arg cam_name)/depth/image_rect_raw*, the topic for the depth image stream
- rgb_inf - *default: /$(arg cam_name)/color/camera_info*, the topic for the RGB camera information
- dep_inf - *default: /$(arg cam_name)/depth/camera_info*, the topic for the depth camera information
- pos_img - *default: /opendr/pose_img*, the name of the topic to which annotated pose images will be published
- cam_frame - *default: sellion_link*, the name of the camera frame
- world_frame - *default: base_link*, the name of the static world frame

Once the node ([/pose](https://github.com/tamlinlove/engage/blob/main/scripts/pose.py)) is running, the following topics will be subscribed/published to:

***Subscribed Topics***
- /camera/color/image_raw (could be different, see launch file arguments), sensor_msgs/Image - the topic of the RGB image stream
- /camera/depth/image_rect_raw (could be different, see launch file arguments), sensor_msgs/Image - the topic of the depth image stream
- /camera/color/camera_info (could be different, see launch file arguments), sensor_msgs/CameraInfo - the topic of the RGB camera information
- /camera/depth/camera_info (could be different, see launch file arguments), sensor_msgs/CameraInfo - the topic of the depth camera information

***Published Topics***
- /opendr/pose_img (could be different, see launch file arguments), sensor_msgs/Image -  the annotated pose image
- /humans/bodies/tracked, hri_msgs/IdsList - the list of random ids for each human body being tracked currently
- /hri_engage/markers, visualization_msgs/Marker - markers for the poses, orientations and velocities of people, mostly for debugging purposes (can be viewed in rviz)
- /humans/bodies/<body_id>/poses, engage_msgs/PoseArrayUncertain - the 3D poses and pose confidences for each body in the world frame
- /humans/bodies/<body_id>/velocity, geometry_msgs/TwistStamped - the velocity vectores for each body in the world frame
- /humans/bodies/<body_id>/body_orientation, geometry_msgs/Vector3Stamped - the orientation of the torso for each body in the world frame
- /humans/bodies/<body_id>/face_orientation, geometry_msgs/Vector3Stamped - the orientation of the face for each body in the world frame
- /humans/bodies/<body_id>/skeleton2d, hri_msgs/Skeleton2D - the 2D skeleton keypoint positions in the camera frame

## High-Level Features

In addition to pose estimation, if you want to calculate higher-level features such as the motion activity, engagement between people and the robot, and group dynamics, you can run the following:

`roslaunch engage perceive.launch`

this takes in the same arguments as the *pose.launch* file, but runs a second node ([/engagement](https://github.com/tamlinlove/engage/blob/main/scripts/engagement.py)) dedicated to calculating higher-level features in addition to the pose estimation node described above. The */engagement* node subscribes/publishes to the following topics:

***Subscribed Topics***
- /humans/bodies/tracked, hri_msgs/IdsList - the list of random ids for each human body being tracked currently
- /humans/bodies/<body_id>/poses, engage_msgs/PoseArrayUncertain - the 3D poses and pose confidences for each body in the world frame
- /humans/bodies/<body_id>/velocity, geometry_msgs/TwistStamped - the velocity vectores for each body in the world frame
- /humans/bodies/<body_id>/body_orientation, geometry_msgs/Vector3Stamped - the orientation of the torso for each body in the world frame
- /humans/bodies/<body_id>/face_orientation, geometry_msgs/Vector3Stamped - the orientation of the face for each body in the world frame

***Published Topics***
- /humans/interactions/engagements, engage_msgs/EngagementValue - tracks the distances, mutual gazes and engagement scores between pairs of people
- /humans/interactions/groups, engage_msgs/Group - the current social groups
- /humans/bodies/<body_id>/engagement_status, engage_msgs/EngagementLevel - the engagement status of the person with the robot, can be UNKNOWN, ENGAGED, DISENGAGED, ENGAGING or DISENGAGING
- /humans/bodies/<body_id>/activity, engage_msgs/MotionActivity - the motion activity of the person, can be NOTHING, WALKING_AWAY, WALKING_TOWARDS or WALKING_PAST
