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

xacro --inorder ${DIR}/urdf/arm.xacro > ${DIR}/urdf/arm.urdf

# rsync -av ${DIR}/ ${CATKIN_WORKSPACE}/manipulator
if [ -d ${CATKIN_WORKSPACE}/manipulator ]; then
	rm -rf ${CATKIN_WORKSPACE}/manipulator
fi

if ! [ -L ${CATKIN_WORKSPACE}/manipulator ]; then
	ln -s ${DIR}/ ${CATKIN_WORKSPACE}/manipulator
fi

# cd ${CATKIN_WORKSPACE} && catkin build manipulator
# source ${CATKIN_ROOT}/devel/setup.bash # TODO: source doesn't work
