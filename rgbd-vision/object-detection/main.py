#!/usr/bin/env python

# Python libs
import sys
import time
import logging

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image

VERBOSE = True

TOPIC_IN = "/camera/rgb/image_color"
TOPIC_OUT = "/darknet/image"


class ObjectDetectorYOLO:
    def __init__(self,
                 classes='./data/yolo3/yolov3.txt',
                 weights='./data/yolo3/yolov3.weights',
                 config='./data/yolo3/yolov3.cfg'):
        self.classes = None
        with open(classes, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.net = cv2.dnn.readNet(weights, config)
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

    def get_output_layers(self, net):
        layer_names = net.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        return output_layers

    def detect(self, image):
        # show image
        # cv2.imshow("Hello", image)
        # cv2.waitKey(2)

        width = image.shape[1]
        height = image.shape[0]
        scale = 0.00392

        # read class names from text file

        # generate different colors for different classes

        # read pre-trained model and config file
        # net = cv2.dnn.readNet(args.weights, args.config)

        # create input blob
        blob = cv2.dnn.blobFromImage(image, scale, (416, 416), (0, 0, 0), True, crop=False)

        # set input blob for the network
        self.net.setInput(blob)

        # run inference through the network
        # and gather predictions from output layers
        outs = self.net.forward(self.get_output_layers(self.net))

        # initialization
        class_ids = []
        confidences = []
        boxes = []
        conf_threshold = 0.5
        nms_threshold = 0.4

        # for each detection from each output layer
        # get the confidence, class id, bounding box params
        # and ignore weak detections (confidence < 0.5)
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = center_x - w / 2
                    y = center_y - h / 2
                    class_ids.append(class_id)
                    confidences.append(float(confidence))
                    boxes.append([x, y, w, h])

        # apply non-max suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)

        # go through the detections remaining
        # after nms and draw bounding box
        for i in indices:
            i = i[0]
            box = boxes[i]
            x = box[0]
            y = box[1]
            w = box[2]
            h = box[3]
            if VERBOSE:
                print "Class: " + self.classes[class_ids[i]] + ", coordinates: (" \
                      + str(round(x)) + "," + str(round(y)) + "),(" + str(round(x + w)) + "," + str(round(y + h)) + ")"


class DarknetImage:
    def __init__(self):
        """Initialize ros publisher, ros subscriber"""
        # topic where we publish
        self.image_pub = rospy.Publisher(TOPIC_OUT, Image, queue_size=1)
        self.bridge = CvBridge()

        # subscribed topic
        self.subscriber = rospy.Subscriber(TOPIC_IN, Image, self.callback,  queue_size = 1)
        if VERBOSE:
            print("subscribed to " + TOPIC_IN)

    def callback(self, ros_data):
        if VERBOSE:
            print('received image of type: "%s"' % ros_data.encoding)
        cv_image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        objectDetector = ObjectDetectorYOLO()
        objectDetector.detect(cv_image)


def main(args):
    """Initializes and cleanup ros node"""
    ic = DarknetImage()
    rospy.init_node('darknet_image', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

