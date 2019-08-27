#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    # rospy.loginfo(rospy.get_name()+" %s ",data.data)
    bridge = CvBridge()
    
    def draw_sidepoints(a, b, grab):
        # left_point = (x,y-y1)
        left_point = (int(a[0]+(a[1]-a[0])*grab), (a[1]+b[1])/2)
        # right_point = (x1,y-y1)
        right_point = (int(b[0]-(a[1]-a[0])*grab), (a[1]+b[1])/2)
        cv2.circle(image, left_point, 2, (0,255,0), 2)
        cv2.circle(image, right_point, 2, (0,255,0), 2)
    
    def draw_box(a,b):
        cv2.rectangle(image, a, b, (255,0,0), 2)
        
    try:
        # image = bridge.imgmsg_to_cv2(data, desired_encoding='mono16')
        # data.encoding = "bgr8"
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        x=255
        y=360
        x1=350
        y1=265
        GRAB_INDEX=0.15
        
        left_point = (x,y)
        right_point = (x1,y1)
        
        
        draw_box(left_point, right_point)
        draw_sidepoints(left_point, right_point, GRAB_INDEX)
        
    except CvBridgeError as e:
        print(e)
    
    # cv2.imwrite("image_try", image)
    cv2.imshow('image', image)
    cv2.waitKey(2)
    
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("camera/rgb/image_color", Image, callback)
    #declares that your node subscribes to the chatter topic which is of type   s   std_msgs.msgs.String
    rospy.spin()
    #keeps your node from exiting until the node has been shutdown
    
    
if __name__ == '__main__':
    listener()