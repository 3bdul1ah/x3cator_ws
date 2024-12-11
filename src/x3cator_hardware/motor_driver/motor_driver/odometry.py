#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from x3cator_msgs.msg import WheelVelocity
from geometry_msgs.msg import Twist, TransformStamped
import math
from nav_msgs.msg import Odometry
import tf2_ros
from tf_transformations import quaternion_from_euler

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')

        self.wheel_radius = self.declare_parameter('wheel_radius', 0.012).value
        self.wheel_distance = self.declare_parameter('wheel_distance', 0.70).value
        self.publish_tf = self.declare_parameter('publish_tf', True).value

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.prev_time = self.get_clock().now()

        self.create_subscription(WheelVelocity, "/encoder_speed", self.odom_callback, 10)
        self.create_subscription(Empty, "/reset_odom", self.reset_odom_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Odometry node is running")
        self.get_logger().info(f"Wheel Distance is set to: {self.wheel_distance:.4f}")
        self.get_logger().info(f"Wheel Radius is set to: {self.wheel_radius:.5f}")

    def reset_odom_callback(self, msg):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        self.prev_time = current_time

        left_vel = msg.left_vel * ((2 * math.pi) / 60) * self.wheel_radius
        right_vel = msg.right_vel * ((2 * math.pi) / 60) * self.wheel_radius

        vx = (right_vel + left_vel) / 2
        vth = (right_vel - left_vel) / self.wheel_distance

        delta_x = (vx * math.cos(self.th)) * dt
        delta_y = (vx * math.sin(self.th)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        odom_quat = quaternion_from_euler(0, 0, self.th)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = current_time.to_msg()
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_footprint"

            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = odom_quat[0]
            transform.transform.rotation.y = odom_quat[1]
            transform.transform.rotation.z = odom_quat[2]
            transform.transform.rotation.w = odom_quat[3]

            self.odom_broadcaster.sendTransform(transform)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Odometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
