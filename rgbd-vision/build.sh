#!/usr/bin/env bash
IFS=':' # space is set as delimiter
read -ra ADDR <<< "$CMAKE_PREFIX_PATH" # str is read into an array as tokens separated by IFS
for i in "${ADDR[@]}"; do # access each element of array
    if [[ "$i" =~ devel? ]]
    then
      echo "CATKIN WS FOLDER WAS FOUND"
      CATKIN="$i"
    fi
done

source /opt/ros/$ROS_DISTRO/setup.bash
source $CATKIN/setup.bash

CATKIN_ROOT="$(cd $(echo $CATKIN) && cd .. && pwd)"
CATKIN_WORKSPACE=${CATKIN_ROOT}/src

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=$(dirname ${DIR})

# rsync -av ${DIR}/object_detection/ ${CATKIN_WORKSPACE}/object_detection
# rsync -av ${DIR}/grasp_detection/ ${CATKIN_WORKSPACE}/grasp_detection
if [ -d ${CATKIN_WORKSPACE}/object_detection ]; then
	rm -rf ${CATKIN_WORKSPACE}/object_detection
fi

if [ -d ${CATKIN_WORKSPACE}/grasp_detection ]; then
	rm -rf ${CATKIN_WORKSPACE}/grasp_detection
fi

if ! [ -L ${CATKIN_WORKSPACE}/object_detection ]; then
	ln -s ${DIR}/object_detection ${CATKIN_WORKSPACE}/object_detection
fi

if ! [ -L ${CATKIN_WORKSPACE}/grasp_detection ]; then
	ln -s ${DIR}/grasp_detection ${CATKIN_WORKSPACE}/grasp_detection
fi
# cd ${CATKIN_WORKSPACE} && catkin build object_detection grasp_detection
# source ${CATKIN_ROOT}/devel/setup.bash # TODO: source doesn't work
