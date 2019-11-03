#!/bin/bash
echo "Building project"
./build.sh
echo "Project was built. Lauching."
roslaunch manipulator manipulator.launch
echo "Launch was successful"
