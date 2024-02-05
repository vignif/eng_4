# Engage

A ros noetic package for planning and executing robot-human elicitations in multi-person unstructured environments. See our [late breaking report at HRI 2024](https://www.iri.upc.edu/groups/perception/#TRAIL_XHRI) to learn more.

![The perception, decision-making and explanation pipeline](/repo/img/teaser.png)

# Installation

This package requires Ubuntu 20.04 and ROS noetic. It also depends on the pyhri and hri_msgs packages for ROS4HRI (installation instructions [here](http://wiki.ros.org/hri/Tutorials/Installation%20of%20a%20ROS4HRI%20environment)), and the [play_motion_msgs](https://github.com/pal-robotics/play_motion) and [pal_interaction_msgs](https://github.com/pal-robotics/pal_msgs) packages from Pal robotics. For pose estimation, this package relies on OpenDR, available [here](https://github.com/opendr-eu/opendr/blob/master/docs/reference/installation.md). Note that this implementation has only been tested with the GPU version of OpenDR, using pytorch 1.13.1+cu117.

For python packages required, see the [requirements.txt](requirements.txt) file. Use of a virtual environment, such as [rosvenv](https://github.com/ARoefer/rosvenv), is highly recommended.

Hardware-wise, this package requires an RGB and depth stream from a camera, such as a realsense D435i. The robot controller scripts have been tested with a [Pal ARI](https://pal-robotics.com/robots/ari/).

# Usage

In this package there are four main scripts: one for <a href="#pose-estimation">pose estimation</a>, one for calculating <a href="#high-level-features">high-level features</a>, one for <a href="#decision-making">calculating decisions</a>, and one for visualising rosbags and explaining decisions post-hoc.

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
- accelerate - *default: False*, if True will use some parameters to improve the pose estimation algorithm's performance

Once the node ([/pose](/scripts/pose.py)) is running, the following topics will be subscribed/published to:

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

In addition to the arguments in the *pose.launch* file, this takes in the following arguments:

- engagement_threshold - *default: 0.55*, the threshold over which someone is considered engaged
- max_angle - *default: PI/2*, the angle over which mutual gaze is 0

Launching this file runs, in addition to the */pose* node, a second node ([/engagement](/scripts/engagement.py)) dedicated to calculating higher-level features in addition to the pose estimation node described above. The */engagement* node subscribes/publishes to the following topics:

***Subscribed Topics***

- /humans/bodies/tracked, hri_msgs/IdsList - the list of random ids for each human body being tracked currently
- /humans/bodies/<body_id>/poses, engage_msgs/PoseArrayUncertain - the 3D poses and pose confidences for each body in the world frame
- /humans/bodies/<body_id>/velocity, geometry_msgs/TwistStamped - the velocity vectors for each body in the world frame
- /humans/bodies/<body_id>/body_orientation, geometry_msgs/Vector3Stamped - the orientation of the torso for each body in the world frame
- /humans/bodies/<body_id>/face_orientation, geometry_msgs/Vector3Stamped - the orientation of the face for each body in the world frame

***Published Topics***

- /humans/interactions/engagements, engage_msgs/EngagementValue - tracks the distances, mutual gazes and engagement scores between pairs of people
- /humans/interactions/groups, engage_msgs/Group - the current social groups
- /humans/bodies/<body_id>/engagement_status, engage_msgs/EngagementLevel - the engagement status of the person with the robot, can be UNKNOWN, ENGAGED, DISENGAGED, ENGAGING or DISENGAGING
- /humans/bodies/<body_id>/activity, engage_msgs/MotionActivity - the motion activity of the person, can be NOTHING, WALKING_AWAY, WALKING_TOWARDS or WALKING_PAST

## Decision-Making

On top of low- and high-level perception, this package also supports a decision-making node ([/decide](/scripts/decide.py)) which listens to the state published by the */pose* and */engagement* nodes and uses this state to make decisions and optionally control a robot. To run the three nodes together, while recording the relevant topics in rosbags, you can run the following:

`roslaunch engage decide.launch exp:=<experiment_name>`

In addition to the arguments taken in by the *perceive.launch* file, this launch file also takes in the following arguments:

- exp - the name of the experiment, used to name the rosbag files
- bag_dir - *default: $(find engage)/rosbags*, the directory to save rosbags in
- robot - *default: True*, if True will send commands to the robot, otherwise will not
- decision_maker - *default: random_robot*, the name of the decision maker (which reads in a state and makes decisions)
- robot_controller - *default: simple_ari_controller*, the name of the robot controller (which converts decisions to robot actions)
- z_offset - *default: 0.3*, the offset above a person's nose which the robot will use as a gaze target (this accounts for a camera placed above the robot's eyes)
- reduced_action_space - *default: True*, if True will reduce the set of actions available for some decision makers
- record_cam - *default: false*, if true will record the raw RGB and depth streams
- language - *default: english*, the language the robot will speak (currently supports english and catalan)
- rosbag_duration - *default: 2*, the number of minutes long each rosbag will be

The topics which the */decide* node may subscribe/publish to are as follows:

***Subscribed Topics***

- /humans/bodies/tracked, hri_msgs/IdsList - the list of random ids for each human body being tracked currently
- /humans/interactions/engagements, engage_msgs/EngagementValue - tracks the distances, mutual gazes and engagement scores between pairs of people
- /humans/interactions/groups, engage_msgs/Group - the current social groups
- /humans/bodies/<body_id>/poses, engage_msgs/PoseArrayUncertain - the 3D poses and pose confidences for each body in the world frame
- /humans/bodies/<body_id>/velocity, geometry_msgs/TwistStamped - the velocity vectors for each body in the world frame
- /humans/bodies/<body_id>/activity, engage_msgs/MotionActivity - the motion activity of the person, can be NOTHING, WALKING_AWAY, WALKING_TOWARDS or WALKING_PAST
- /humans/bodies/<body_id>/engagement_status, engage_msgs/EngagementLevel - the engagement status of the person with the robot, can be UNKNOWN, ENGAGED, DISENGAGED, ENGAGING or DISENGAGING

***Published Topics***

Depending on the type of decision maker, a different decision and state message type will be employed

- heuristic
  - /hri_engage/decisions, engage_msgs/HeuristicDecision - the decision made
  - /hri_engage/decision_states, engage_msgs/HeuristicStateDecision - the state used to make the decision
- random_robot
  - /hri_engage/decisions, engage_msgs/RobotDecision - the decision made
  - /hri_engage/decision_states, engage_msgs/RobotStateDecision - the state used to make the decision

If decisions are executed using one of the controller scripts written for the Pal ARI, then the following topics are also published to:

- /play_motion/goal, play_motion_msgs/PlayMotionActionGoal - the motion instructions for the robot
- /look_at, geometry_msgs/PointStamped - the gaze target for the robot
- /tts/goal, pal_interaction_msgs/TtsActionGoal - the text-to-speech instructions for the robot
