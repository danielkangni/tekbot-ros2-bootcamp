#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
[ -f /ws/out/install/setup.bash ] && source /ws/out/install/setup.bash
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/usr/share/gazebo-11/models"
exec "$@"
