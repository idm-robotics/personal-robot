#!/usr/bin/env python

from object_detection.msg import DetectedObjectArray


class Detector:
    def detect(self, image, header) -> DetectedObjectArray:
        raise NotImplementedError("detect method not implemented")
