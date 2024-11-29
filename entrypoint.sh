#!/bin/bash
# apt-get update

pwd

source /opt/ros/noetic/setup.bash

catkin build

source devel/setup.bash

# rosparam delete /use_sim_time

roslaunch engage perceive.launch 


# Keep the container running
exec "$@"

