#!/usr/bin/env python

from object_detection.msg import DetectedObjectArray


class Detector:
    def detect(self, image) -> DetectedObjectArray:
        raise NotImplementedError("detect method not implemented")
