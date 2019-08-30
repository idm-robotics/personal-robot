from typing import Tuple

import image_geometry
from sensor_msgs.msg import CameraInfo


class PointCloudConverter:
    def __init__(self, camera_info: CameraInfo, unit_scaling: float = 0.001):
        self.unit_scaling = unit_scaling
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(camera_info)

    def convert(self, x: int, y: int, depth: int) -> Tuple[float, float, float]:
        pointcloud_x = (x - self.cam_model.cx()) * depth * self.unit_scaling / self.cam_model.fx()
        pointcloud_y = (y - self.cam_model.cy()) * depth * self.unit_scaling / self.cam_model.fy()
        pointcloud_z = depth * self.unit_scaling
        return pointcloud_x, pointcloud_y, pointcloud_z
