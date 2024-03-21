# Engage

A ros noetic package for executing and explaining robot-human elicitations in multi-person unstructured environments. See our [late breaking report at HRI 2024](https://www.iri.upc.edu/groups/perception/#TRAIL_XHRI) to learn more.

![The perception, decision-making and explanation pipeline](/repo/img/teaser.png)

# Installation

This package requires Ubuntu 20.04 and ROS noetic. It also depends on the pyhri and hri_msgs packages for ROS4HRI (installation instructions [here](http://wiki.ros.org/hri/Tutorials/Installation%20of%20a%20ROS4HRI%20environment)), and the [play_motion_msgs](https://github.com/pal-robotics/play_motion) and [pal_interaction_msgs and pal_web_msgs](https://github.com/pal-robotics/pal_msgs) packages from Pal robotics. For pose estimation, this package relies on OpenDR, available [here](https://github.com/opendr-eu/opendr/blob/master/docs/reference/installation.md). Note that this implementation has only been tested with the GPU version of OpenDR, using pytorch 1.13.1+cu117.

For python packages required, see the [requirements.txt](requirements.txt) file. Use of a virtual environment, such as [rosvenv](https://github.com/ARoefer/rosvenv), is highly recommended.

Hardware-wise, this package requires an RGB and depth stream from a camera, such as a realsense D435i. The robot controller scripts have been tested with a [Pal ARI](https://pal-robotics.com/robots/ari/).

# Usage
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
- /humans/bodies/positions, engage_msgs/PeoplePositions - the 3d and 2d positions of a point for each person, used mainly for labelling

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
- reduced_action_space - *default: True*, if True will reduce the set of actions available for some decision makersç
- wait_time - *default: 5*, mean time to wait between decisions
- wait_deviation - *default: 1*, standard deviation of wait times, if 0 will be fixed
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

## Live Explanations

If you wish to run the experiments detailed in our RO-MAN paper, you can run the following:

`roslaunch engage prediction_experiment.launch exp:=<experiment_name> language:=catalan`

This runs all of the aforementioned nodes, as well as one that generates explanations in real-time and one that manages the tablet interface.

In addition to the previously mentioned arguments, this launch file also takes in the following:

- explain_image_buffer - *default: 5*, the number of image messages to be saved in a buffer for use in the prediction game
- explainer - *default: counterfactual*, the explanation generation module, currently only *counterfactual* is supported, but there is a LIME explainer with limited functionality
- groups - *default: all*, which experimental conditions to use. *all* uses groups C, E and CF. For single groups, use *control* (C), *nocf* (E) or *full* (CF). *nomid*, for groups C and CF, is also supported.
- image_mode - *default: rgb*, if *rgb* will display RGB images to participants, and if *pose* will use the annotated pose image
- decision_threshold - *default: 1*, the distance within which the decision-making will be paused and the predicition test behaviour will take over
- input_timeout_duration - *default: 60*, the number of seconds to wait after each input before the test times out
- start_timeout_duration - *default: 15*, a shorter timeout duration, used on the first page if nobody starts the test

In addition to the */pose*, */engagement* and */decide* nodes, this launch file also spins up */live_explainer* and */prediction_experiment_manager* nodes. The */live_explainer* node has the following subscribed/published topics:

***Subscribed Topics***
- /camera/color/image_raw (could be different, see launch file arguments), sensor_msgs/Image - the topic of the RGB or pose image stream
- /humans/bodies/positions, engage/PeoplePositions - the positions of each person, for use in labelling the prediction test image
- /hri_engage/decision_states, message type varies depending on decision-maker as above - the decisions and states used to generate explanations
- /out_explanation, explanation_msgs/Explainability - the topic for final results, listened to in order to change group and explanation variable

***Published Topics***

- /input_explanation, explanation_msgs/Explainability - the topic for explanations which the tablet listens to

The */prediction_experiment_manager* on the other hand has the following subscribed/published topics:

***Subscribed Topics***
- /hri_engage/decision_states, message type varies depending on decision-maker as above - the decisions and states used to check if anyone is close to the robot
- /page_name, std_msgs/String - the name of the current tablet page
- /user_input, pal_interaction_msgs/Input - the input on the tablet

***Published Topics***

- /web/go_to, pal_web_msgs/WebGoTo - for controlling the tablet
