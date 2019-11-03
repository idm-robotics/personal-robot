import os

import rospy
import tensorflow as tf
from object_detection.msg import DetectedObject
from object_detection.msg import DetectedObjectArray

import coco
import mrcnn.model as modellib
from detector import Detector

# Uncomment lines below if you want to use CPU instead of GPU
os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = ""


def get_rel_project_path(path: str) -> str:
    return os.path.join(os.path.dirname(__file__), path)


class InferenceConfig(coco.CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1


class MaskRCNNDetector(Detector):
    def __init__(self,
                 weights=get_rel_project_path('../data/mask_rcnn/mask_rcnn_coco.h5'),
                 model_dir=get_rel_project_path('../logs')):
        config = InferenceConfig()
        self.model = modellib.MaskRCNN(mode='inference', model_dir=model_dir, config=config)
        self.model.load_weights(weights, by_name=True)
        self.model.keras_model._make_predict_function()
        self.class_names = ['BG', 'person', 'bicycle', 'car', 'motorcycle', 'airplane',
                            'bus', 'train', 'truck', 'boat', 'traffic light',
                            'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird',
                            'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear',
                            'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie',
                            'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                            'kite', 'baseball bat', 'baseball glove', 'skateboard',
                            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
                            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                            'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza',
                            'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed',
                            'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
                            'keyboard', 'cell phone', 'microwave', 'oven', 'toaster',
                            'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors',
                            'teddy bear', 'hair drier', 'toothbrush']

    def detect(self, image, header):
        start_time = rospy.Time.now()
        results = self.model.detect([image], verbose=1)
        rospy.loginfo(f"Tensorflow gpu available: {tf.test.is_gpu_available()}")
        rospy.loginfo("net.forward:%6.3f" % (rospy.Time.now() - start_time).to_sec())

        detected_object_array = DetectedObjectArray()
        detected_object_array.header = header
        detected_object_array.data = []

        result = results[0]
        for i, class_id in enumerate(result['class_ids']):
            # score: result['scores'][i]
            detected_object = DetectedObject()
            detected_object.category = self.class_names[class_id]
            detected_object.top = result['rois'][i][0]
            detected_object.left = result['rois'][i][1]
            detected_object.bottom = result['rois'][i][2]
            detected_object.right = result['rois'][i][3]
            detected_object_array.data.append(detected_object)
            rospy.loginfo(f"Class: {detected_object.category}, coordinates: [{detected_object.top}, "
                          f"{detected_object.left}, {detected_object.bottom}, {detected_object.right}]")

        return detected_object_array
