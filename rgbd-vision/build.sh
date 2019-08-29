#!/usr/bin/env bash

CATKIN_WORKSPACE=$HOME/ws_moveit/src # Replace $HOME/ws_moveit/src to your catkin workspace directory!

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=$(dirname ${DIR})

rsync -av ${DIR}/object-detection/ ${CATKIN_WORKSPACE}/object-detection
rsync -av ${DIR}/grasp-detection/ ${CATKIN_WORKSPACE}/grasp-detection
cd ${CATKIN_WORKSPACE} && catkin build object-detection grasp-detection