import image_geometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo


class PointCloudConverter:
    # TODO: investigate correct value of unit_scaling (should it be 0.001 ?)
    def __init__(self, camera_info: CameraInfo, unit_scaling: float = 1.0):
        self.unit_scaling = unit_scaling
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(camera_info)

    def convert(self, x: int, y: int, depth: float) -> Point:
        pointcloud_x = (x - self.cam_model.cx()) * depth * self.unit_scaling / self.cam_model.fx()
        pointcloud_y = (y - self.cam_model.cy()) * depth * self.unit_scaling / self.cam_model.fy()
        pointcloud_z = depth * self.unit_scaling
        return Point(pointcloud_x, pointcloud_y, pointcloud_z)
