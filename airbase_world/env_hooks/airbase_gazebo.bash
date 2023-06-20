#! /bin/bash

source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/aribase_world/models/:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/airbase_world/models/:${CATKIN_ENV_HOOK_WORKSPACE}/../src/airbase_world/worlds/:${GAZEBO_RESOURCE_PATH}"