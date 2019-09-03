#!/usr/bin/env python

import os
import cv2
import numpy as np
from object_detection.msg import DetectedObject
from object_detection.msg import DetectedObjectArray
from detector import Detector

VERBOSE = True


def get_rel_project_path(path: str) -> str:
    return os.path.join(os.path.dirname(__file__), path)


class YOLODetector(Detector):
    def __init__(self,
                 classes=get_rel_project_path('../data/yolo3/yolov3.txt'),
                 weights=get_rel_project_path('../data/yolo3/yolov3.weights'),
                 config=get_rel_project_path('../data/yolo3/yolov3.cfg')):
        self.classes = None
        with open(classes, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.net = cv2.dnn.readNet(weights, config)
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

    @staticmethod
    def get_output_layers(net):
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
        detected_object_array = DetectedObjectArray()
        detected_object_array.data = []

        for i in indices:
            i = i[0]
            box = boxes[i]
            x = round(box[0])
            y = round(box[1])
            w = round(box[2])
            h = round(box[3])
            category = self.classes[class_ids[i]]
            if VERBOSE:
                print("Class: " + category + ", coordinates: ("
                      + str(x) + "," + str(y) + "),(" + str(x + w) + "," + str(y + h) + ")")
            detected_object = DetectedObject()
            detected_object.category = category
            detected_object.top = y + h
            detected_object.bottom = y
            detected_object.left = x
            detected_object.right = x + w
            detected_object_array.data.append(detected_object)

        return detected_object_array
