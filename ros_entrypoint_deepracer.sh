#!/bin/bash
set -e

export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:{$GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=/app/deepracer_nav2_ws/aws-deepracer/deepracer_gazebo/models 
export GAZEBO_PLUGIN_PATH=/app/deepracer_nav2_ws/aws-deepracer/deepracer_gazebo/build 

source /opt/ros/foxy/setup.bash
source /app/deepracer_nav2_ws/aws-deepracer/install/setup.bash

exec "$@"
