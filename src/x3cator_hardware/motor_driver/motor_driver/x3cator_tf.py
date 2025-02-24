#!/usr/bin/env python3
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from rclpy.node import Node

class RobotTransformationPublisherNode(Node):
    def __init__(self):
        super().__init__('robot_transformation_publisher_node')

        self.transformation_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        world_to_base_link_transformation = TransformStamped()
        world_to_base_link_transformation.header.frame_id = "world"
        world_to_base_link_transformation.child_frame_id = "base_link"
        world_to_base_link_transformation.transform.translation.x = 1.5
        world_to_base_link_transformation.transform.translation.y = 1.
        world_to_base_link_transformation.transform.translation.z = 0.0
        quaternion_for_world_to_base_link = quaternion_from_euler(0, 0, 0)
        world_to_base_link_transformation.transform.rotation.x = quaternion_for_world_to_base_link[0]
        world_to_base_link_transformation.transform.rotation.y = quaternion_for_world_to_base_link[1]
        world_to_base_link_transformation.transform.rotation.z = quaternion_for_world_to_base_link[2]
        world_to_base_link_transformation.transform.rotation.w = quaternion_for_world_to_base_link[3]

        base_link_to_lidar_transformation = TransformStamped()
        base_link_to_lidar_transformation.header.frame_id = "base_link"
        base_link_to_lidar_transformation.child_frame_id = "lidar_link"
        base_link_to_lidar_transformation.transform.translation.x = 0.437
        base_link_to_lidar_transformation.transform.translation.y = 0.0
        base_link_to_lidar_transformation.transform.translation.z = 0.429
        quaternion_for_base_link_to_lidar = quaternion_from_euler(0, 0, 0)
        base_link_to_lidar_transformation.transform.rotation.x = quaternion_for_base_link_to_lidar[0]
        base_link_to_lidar_transformation.transform.rotation.y = quaternion_for_base_link_to_lidar[1]
        base_link_to_lidar_transformation.transform.rotation.z = quaternion_for_base_link_to_lidar[2]
        base_link_to_lidar_transformation.transform.rotation.w = quaternion_for_base_link_to_lidar[3]

        base_link_to_camera_transformation = TransformStamped()
        base_link_to_camera_transformation.header.frame_id = "base_link"
        base_link_to_camera_transformation.child_frame_id = "camera_link"
        base_link_to_camera_transformation.transform.translation.x = 0.53
        base_link_to_camera_transformation.transform.translation.y = 0.0
        base_link_to_camera_transformation.transform.translation.z = 0.13
        quaternion_for_base_link_to_camera = quaternion_from_euler(0, 0, 0)
        base_link_to_camera_transformation.transform.rotation.x = quaternion_for_base_link_to_camera[0]
        base_link_to_camera_transformation.transform.rotation.y = quaternion_for_base_link_to_camera[1]
        base_link_to_camera_transformation.transform.rotation.z = quaternion_for_base_link_to_camera[2]
        base_link_to_camera_transformation.transform.rotation.w = quaternion_for_base_link_to_camera[3]

        self.transformation_broadcaster.sendTransform([world_to_base_link_transformation, base_link_to_lidar_transformation, base_link_to_camera_transformation])

def main(args=None):
    rclpy.init(args=args)
    robot_transformation_publisher_node = RobotTransformationPublisherNode()
    rclpy.spin_once(robot_transformation_publisher_node)
    robot_transformation_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
