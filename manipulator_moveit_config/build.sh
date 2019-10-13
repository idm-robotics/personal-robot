#!/usr/bin/env bash
CATKIN_ROOT=$HOME/ws_moveit # Replace $HOME/ws_moveit/src to your catkin workspace directory!
CATKIN_WORKSPACE=${CATKIN_ROOT}/src 

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=$(dirname ${DIR})

rsync -av ${DIR}/ ${CATKIN_WORKSPACE}/manipulator_moveit_config
cd ${CATKIN_WORKSPACE} && catkin build manipulator_moveit_config
source ${CATKIN_ROOT}/devel/setup.bash # TODO: source doesn't work