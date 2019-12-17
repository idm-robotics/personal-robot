#!/usr/bin/env bash
declare -a PACKAGES=("manipulator"
					 "manipulator_moveit_config" 
					 "object_detection" 
					 "grasp_detection")

declare -a RGBD_VISION=("object_detection" 
				 		"grasp_detection")

# get catkin folder
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

xacro --inorder ${DIR}/manipulator/urdf/arm.xacro > ${DIR}/manipulator/urdf/arm.urdf

# check if actual folder exists and delete it
# if folder doesnt have symbolic link then create it
for i in "${PACKAGES[@]}"; do
	FOLDER="$i"
	if [[ " ${RGBD_VISION[@]} " =~ " ${i} " ]]; then
	    FOLDER="rgbd-vision/$i"
	fi

	if [ -d ${CATKIN_WORKSPACE}/$i ]; then
		rm -rf ${CATKIN_WORKSPACE}/$i
	fi

	if ! [ -L ${CATKIN_WORKSPACE}/$i ]; then
		ln -s ${DIR}/$FOLDER ${CATKIN_WORKSPACE}/$i
	fi
done

# build all packages
cd ${CATKIN_WORKSPACE} && catkin build "${PACKAGES[@]}"