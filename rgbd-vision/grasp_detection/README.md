## Grasp detection
### Recording to rosbag
To record rosbag launch openni node
```bash
roslaunch openni_launch openni.launch depth_registration:=true
```
Record to rosbag
```bash
rosbag record cera/depth_registered/image_raw camera/depth_registered/camera_info camera/rgb/image_raw camera/rgb/camera_info --limit=100 -O kinect
```
### Launch python node on our rosbag
Launch openni driver
```bash
roslaunch openni_launch openni.launch load_driver:=false
```
Put python script to the node directory. Build package with python code
```bash
catkin build xxx
```
Because of catkin build we need to change permissions and then we can run python script
```bash
chmod +x python_subscriber_node.py 
rosrun moveit_tutorials python_subscriber_node.py
```
Launch rosbag replay after python script
```bash
rosbag play --clock kinect.bag
```

To check that rosbag works launch driver and play recording. In rviz we need to add pointcloud2 node and change fixed frame to camera_link and topic to cameta/depth_registered/points
```bash
rosrun rviz rviz
```

## Changes
Changes in library filter_messages (row 286, zip to list[zip])

Added `from functools import reduce` to the library.

Also pointcloud_conversion receives messages from camera_info and returns
x, y, z points in PointCloud format.

Now python_subscriber_node.py both publishes and subscribes to nodes and
it should be able to add grasp points to the cup in the rosbag file.




