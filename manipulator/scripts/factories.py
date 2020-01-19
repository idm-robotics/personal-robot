from grasp_evaluator import BaseMovement
from moveit_msgs.msg import Grasp as GraspObject
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import moveit_msgs
from trajectory_msgs.msg import JointTrajectory
from gripper.msgs.msg import GripperTranslation


class GraspFactory:
    def build(self, grasp_coordinates):
        grasp = GraspObject()
        grasp.id = 'grasp'

        # Pose of the end effector in which it should attempt grasping.
        grasp.grasp_pose = PoseStamped()
        # TODO: this should be pose of end_eddector so be careful
        grasp.grasp_pose = BaseMovement.__get_transform_pose(grasp_coordinates)

        # This defines the trajectory position of the joints in the end
        # effector group before we go in for the grasp.
        grasp.pre_grasp_posture = JointTrajectory()
        # Can I extract it from pose?

        # This defines the trajectory position of the joints in the end
        # effector group for grasping the object.
        grasp.grasp_posture = JointTrajectory()
        # Can I extract it from pose?

        # This is used to define the direction from which to approach the
        # object and the distance to travel.
        grasp.pre_grasp_approach = GripperTranslation()
        # /* Defined with respect to frame_id */
        # grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        # /* Direction is set as positive x axis */
        grasp.pre_grasp_approach.direction.vector.x = 1.0
        grasp.pre_grasp_approach.min_distance = 0.095
        grasp.pre_grasp_approach.desired_distance = 0.115;

        # This is used to define the direction in which to move once the
        # object is grasped and the distance to travel.
        grasp.post_grasp_retreat = GripperTranslation()
        # /* Defined with respect to frame_id */
        # grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
        # /* Direction is set as positive z axis */
        # grasp.post_grasp_retreat.direction.vector.z = 1.0;
        # grasp.post_grasp_retreat.min_distance = 0.1;
        # grasp.post_grasp_retreat.desired_distance = 0.25;

        # This is used to define the direction in which to move once the
        # object is placed at some location and the distance to travel.
        grasp.post_place_retreat = GripperTranslation()

        return grasp

class ObjectFactory:
    @staticmethod
    def __get_coordinates_for_object(grasp_coordinates=None):
        if not grasp_coordinates:
            return [0.5, 0 ,0.5]
        else:
            left_point = np.array([grasp_coordinates.left_point.x,
                                   grasp_coordinates.left_point.y,
                                   grasp_coordinates.left_point.z])
            right_point = np.array([grasp_coordinates.right_point.x,
                                    grasp_coordinates.right_point.y,
                                    grasp_coordinates.right_point.z])
            object_position = (left_point + right_point) / 2

            return [object_position[0], object_position[1], object_position[2]]

    @staticmethod
    def __get_dimensions():
        # estimate size of an object
        return [0.02, 0.02, 0.2]

    def build(self):
        # collision_object.header.frame_id = "panda_link0"
        collision_object.id = "object"

        # /* Define the primitive and its dimensions. */
        collision_object.primitives.resize(1)
        collision_object.primitives[0].type = collision_objects[1].primitives[0].BOX
        collision_object.primitives[0].dimensions.resize(3)
        dimensions = __get_dimensions()
        collision_object.primitives[0].dimensions[0] = dimensions[0]
        collision_object.primitives[0].dimensions[1] = dimensions[1]
        collision_object.primitives[0].dimensions[2] = dimensions[2]

        # /* Define the pose of the object. */
        collision_object.primitive_poses.resize(1)
        coordinates = __get_coordinates_for_object(grasp_coordinates)
        collision_object.primitive_poses[0].position.x = coordinates[0]
        collision_object.primitive_poses[0].position.y = coordinates[1]
        collision_object.primitive_poses[0].position.z = coordinates[2]

        return collision_object
