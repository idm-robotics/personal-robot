#!/usr/bin/env bash

PYTHON_VERSION=3.6 # Feel free to change it to 3.7 or upper

DIR="$( cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd )"
PROJECT_DIR=$(dirname ${DIR})

echo "Project directory: " ${PROJECT_DIR}

[[ ! -f ${DIR}/object-detection/data/yolo3/yolov3.weights ]] \
    && wget https://pjreddie.com/media/files/yolov3.weights -O ${DIR}/data/yolo3/yolov3.weights

echo "Creating venv..."
cd ${PROJECT_DIR}
[[ ! -d ${PROJECT_DIR}/venv ]] && python${PYTHON_VERSION} -m venv venv
echo "...Done"

echo "Activating venv..."
. ${PROJECT_DIR}/venv/bin/activate
echo "...Done"

echo "Installing python dependencies..."
pip install --upgrade pip
pip install -r ${DIR}/requirements.txt
echo "...Done"

echo "Downloading and installing cv_bridge..."
sudo apt-get install python-catkin-tools python3-dev python3-catkin-pkg-modules \
                     python3-numpy python3-yaml \
                     ros-${ROS_DISTRO}-cv-bridge

rm -rf $HOME/tmp_catkin_ws
mkdir $HOME/tmp_catkin_ws && cd $HOME/tmp_catkin_ws
catkin init
catkin config -DPYTHON_EXECUTABLE=${PROJECT_DIR}/venv/bin/python \
              -DPYTHON_INCLUDE_DIR=/usr/include/python${PYTHON_VERSION}m \
              -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython${PYTHON_VERSION}m.so
catkin config --install

git clone -b ${ROS_DISTRO} https://github.com/ros-perception/vision_opencv.git src/vision_opencv

catkin build cv_bridge || sed -i 's/Boost REQUIRED python3/Boost REQUIRED python-py35/' \
    $HOME/tmp_catkin_ws/src/vision_opencv/cv_bridge/CMakeLists.txt && catkin build cv_bridge
echo "...Done"

echo "Copying cv_bridge to venv..."
cp -r $HOME/tmp_catkin_ws/install/lib/python3/dist-packages/* \
      ${PROJECT_DIR}/venv/lib/python${PYTHON_VERSION}/site-packages/
echo "...Done"

echo "Removing cv_bridge from tmp directory..."
rm -rf $HOME/tmp_catkin_ws
echo "...Done"

echo "Removing cv_bridge from ros..."
sudo rm -rf /opt/ros/${ROS_DISTRO}/lib/python2.7/dist-packages/cv_bridge*
echo "...Done"

echo "Installing has finished successfully! Look at build.sh next"
