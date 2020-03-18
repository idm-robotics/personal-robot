#!/bin/bash

set -e

ufw enable
ufw allow from $1

export ROS_MASTER_URI="http://$1:11311"
# export ROS_IP=`hostname -I`

rostopic list
while [ $? -eq 0 ]; do
  rostopic list
  sleep 1
done

rosrun object_detection main.py