#!/usr/bin/env bash
CATKIN_ROOT=$HOME/ws_moveit # Replace $HOME/ws_moveit/src to your catkin workspace directory!
CATKIN_WORKSPACE=${CATKIN_ROOT}/src 

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=$(dirname ${DIR})

rsync -av ${DIR}/object_detection/ ${CATKIN_WORKSPACE}/object_detection
rsync -av ${DIR}/grasp_detection/ ${CATKIN_WORKSPACE}/grasp_detection
cd ${CATKIN_WORKSPACE} && catkin build object_detection grasp_detection
source ${CATKIN_ROOT}/devel/setup.bash