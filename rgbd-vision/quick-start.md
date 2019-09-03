## RGBD Vision Quick start

### Prerequisites

Use Ubuntu 16 or 18, python 3.6+. Also you need kinect 360

Expected, that you've installed ros (kinetic or melodic), openni_launch and catkin workspace is created

TODO: put here links to ros guides

### Environment preparation

1. Specify your python version in `install.sh`
1. Run `install.sh`. It should create python 3 venv, install all dependencies

### PyCharm Settings

If you use PyCharm you can use venv in interpreter settings. Also you can add additional path 
`/opt/ros/melodic/lib/python2.7/dist-packages/` to interpreter settings. Autocomplete will work fine after that.

TODO: add screenshots

### Run

1. Activate venv `. venv/bin/activate`
1. Specify your catkin workspace directory in `build.sh`
1. Run `build.sh`. It will create symlinks in your catkin workspace and build project
1. Connect kinect
1. Run `roslaunch openni_launch openni.launch depth_registration:=true`
1. Open new terminal and finally run your node `rosrun object_detection main.py`


**If you have errors with opencv, you may need to remove cv2.so from `/opt/ros/melodic/lib/python2.7/dist-packages/`**

