#!/usr/bin/env python3
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from rclpy.node import Node

class RobotTFPublisher(Node):
    def __init__(self):
        super().__init__('robot_tf_publisher')

        self.br = tf2_ros.StaticTransformBroadcaster(self)

        base_trans = TransformStamped()
        base_trans.header.frame_id = "base_footprint"
        base_trans.child_frame_id = "base_link"
        base_trans.transform.translation.x = 0.0
        base_trans.transform.translation.y = 0.0
        base_trans.transform.translation.z = 0.3
        q = quaternion_from_euler(0, 0, 0)
        base_trans.transform.rotation.x = q[0]
        base_trans.transform.rotation.y = q[1]
        base_trans.transform.rotation.z = q[2]
        base_trans.transform.rotation.w = q[3]

        imu_trans = TransformStamped()
        imu_trans.header.frame_id = "base_link"
        imu_trans.child_frame_id = "imu_link"
        imu_trans.transform.translation.x = 0.432
        imu_trans.transform.translation.y = 0.125
        imu_trans.transform.translation.z = 0.428
        q = quaternion_from_euler(0, 0, 0)
        imu_trans.transform.rotation.x = q[0]
        imu_trans.transform.rotation.y = q[1]
        imu_trans.transform.rotation.z = q[2]
        imu_trans.transform.rotation.w = q[3]

        lidar_trans = TransformStamped()
        lidar_trans.header.frame_id = "base_link"
        lidar_trans.child_frame_id = "lidar_link"  
        lidar_trans.transform.translation.x = 0.437
        lidar_trans.transform.translation.y = 0.0
        lidar_trans.transform.translation.z = 0.429
        q = quaternion_from_euler(0, 0, 0)
        lidar_trans.transform.rotation.x = q[0]
        lidar_trans.transform.rotation.y = q[1]
        lidar_trans.transform.rotation.z = q[2]
        lidar_trans.transform.rotation.w = q[3]

        cam_trans = TransformStamped()
        cam_trans.header.frame_id = "base_link"
        cam_trans.child_frame_id = "camera_link"  
        cam_trans.transform.translation.x = 0.53
        cam_trans.transform.translation.y = 0.0
        cam_trans.transform.translation.z = 0.13
        q = quaternion_from_euler(0, 0, 0)
        cam_trans.transform.rotation.x = q[0]
        cam_trans.transform.rotation.y = q[1]
        cam_trans.transform.rotation.z = q[2]
        cam_trans.transform.rotation.w = q[3]

        left_wheel_trans = TransformStamped()
        left_wheel_trans.header.frame_id = "base_link"
        left_wheel_trans.child_frame_id = "left_wheel"
        left_wheel_trans.transform.translation.x = 0.0
        left_wheel_trans.transform.translation.y = 0.4
        left_wheel_trans.transform.translation.z = -0.212
        q = quaternion_from_euler(0, 0, 0)
        left_wheel_trans.transform.rotation.x = q[0]
        left_wheel_trans.transform.rotation.y = q[1]
        left_wheel_trans.transform.rotation.z = q[2]
        left_wheel_trans.transform.rotation.w = q[3]

        right_wheel_trans = TransformStamped()
        right_wheel_trans.header.frame_id = "base_link"
        right_wheel_trans.child_frame_id = "right_wheel"
        right_wheel_trans.transform.translation.x = 0.0
        right_wheel_trans.transform.translation.y = -0.4
        right_wheel_trans.transform.translation.z = -0.212
        q = quaternion_from_euler(0, 0, 0)
        right_wheel_trans.transform.rotation.x = q[0]
        right_wheel_trans.transform.rotation.y = q[1]
        right_wheel_trans.transform.rotation.z = q[2]
        right_wheel_trans.transform.rotation.w = q[3]        

        
        # rtk1_trans = TransformStamped()
        # rtk1_trans.header.frame_id = "base_link"
        # rtk1_trans.child_frame_id = "gps"
        # rtk1_trans.transform.translation.x = -0.290
        # rtk1_trans.transform.translation.y = -0.209
        # rtk1_trans.transform.translation.z = 1.062
        # q = quaternion_from_euler(0, 0, 0)
        # rtk1_trans.transform.rotation.x = q[0]
        # rtk1_trans.transform.rotation.y = q[1]
        # rtk1_trans.transform.rotation.z = q[2]
        # rtk1_trans.transform.rotation.w = q[3]

        # rtk2_trans = TransformStamped()
        # rtk2_trans.header.frame_id = "base_link"
        # rtk2_trans.child_frame_id = "gps2"
        # rtk2_trans.transform.translation.x = -0.290
        # rtk2_trans.transform.translation.y = 0.209
        # rtk2_trans.transform.translation.z = 1.062
        # q = quaternion_from_euler(0, 0, 0)
        # rtk2_trans.transform.rotation.x = q[0]
        # rtk2_trans.transform.rotation.y = q[1]
        # rtk2_trans.transform.rotation.z = q[2]
        # rtk2_trans.transform.rotation.w = q[3]

        # laser_trans = TransformStamped()
        # laser_trans.header.frame_id = "base_link"
        # laser_trans.child_frame_id = "laser_link"
        # laser_trans.transform.translation.x = 0.437
        # laser_trans.transform.translation.y = 0.0
        # laser_trans.transform.translation.z = 0.429
        # q = quaternion_from_euler(0, 0, 0)
        # laser_trans.transform.rotation.x = q[0]
        # laser_trans.transform.rotation.y = q[1]
        # laser_trans.transform.rotation.z = q[2]
        # laser_trans.transform.rotation.w = q[3]


        self.br.sendTransform([base_trans, imu_trans, lidar_trans, cam_trans, right_wheel_trans, left_wheel_trans])

def main(args=None):
    rclpy.init(args=args)
    robot_tf_publisher = RobotTFPublisher()
    rclpy.spin(robot_tf_publisher)
    robot_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()