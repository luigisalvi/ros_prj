#!/bin/bash
set -e

echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source /app/deepracer_nav2_ws/aws-deepracer/install/setup.bash" >> ~/.bashrc

echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:{$GAZEBO_RESOURCE_PATH}" >> ~/.bashrc

exec "$@"
