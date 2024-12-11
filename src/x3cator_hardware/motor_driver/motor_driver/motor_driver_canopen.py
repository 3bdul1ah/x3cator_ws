#!/usr/bin/env python3
import os
import time
import canopen
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from x3cator_msgs.msg import WheelVelocity

os.system('sudo ip link set can0 up type can bitrate 250000')
os.system('sudo ifconfig can0 txqueuelen 10000')

class CanOpenNode(Node):
    def __init__(self):
        super().__init__('motor_driver_canopen')
        time.sleep(1)
        self.die = 0
        self.speed = 1
        self.can_node_id = 2
        self.network = canopen.Network()
        self.network.connect(channel="can0", bustype="socketcan")
        self.node = self.network.add_node(self.can_node_id)

        self.last_time = self.get_clock().now()

        self.create_timer(0.1, self.read_encoder_data)

        self.vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.joystick, 10)
        self.speed_subscriber = self.create_subscription(Int8, "/cmd_speed", self.setSpeed, 10)
        self.encoder_speed_pub = self.create_publisher(WheelVelocity, "/encoder_speed", 10)

    def joystick(self, msg):
        linear = -msg.linear.x * 600.0 * self.speed
        angular = -msg.angular.z * 180.0
        self.send_can_message(linear, angular)

    def send_can_message(self, linear, angular):
        linear_bytes = int(linear).to_bytes(4, 'little', signed=True)
        angular_bytes = int(angular).to_bytes(4, 'little', signed=True)

        success_angular = self.send_can_message_with_retry(self.node, 0x2000, 1, angular_bytes)
        success_linear = self.send_can_message_with_retry(self.node, 0x2000, 2, linear_bytes)

        if not success_linear or not success_angular:
            self.get_logger().error("Sending velocities failed.")

    def send_can_message_with_retry(self, node, index, subindex, data):
        MAX_RETRIES = 3
        RETRY_DELAY = 0.1
        for attempt in range(MAX_RETRIES):
            try:
                node.sdo.download(index, subindex, data)
                return True
            except canopen.SdoCommunicationError as e:
                self.get_logger().warn(f"SDO communication error: {e}. Attempt {attempt + 1}/{MAX_RETRIES}")
                time.sleep(RETRY_DELAY)
        self.get_logger().err(f"Failed to send CAN message after {MAX_RETRIES} retries.")
        return False

    def read_encoder_data(self):
        MAX_RETRIES = 3
        RETRY_DELAY = 0.1
        for attempt in range(MAX_RETRIES):
            try:
                encoder_speed_ch1 = self.node.sdo.upload(0x2103, 0x01)
                encoder_speed_ch1_value = int.from_bytes(encoder_speed_ch1, byteorder='little', signed=True)

                encoder_speed_ch2 = self.node.sdo.upload(0x2103, 0x02)
                encoder_speed_ch2_value = int.from_bytes(encoder_speed_ch2, byteorder='little', signed=True)

                return encoder_speed_ch1_value, encoder_speed_ch2_value
            except canopen.SdoCommunicationError as e:
                self.get_logger().warn(f"Failed to read encoder data: {e}. Attempt {attempt + 1}/{MAX_RETRIES}")
                time.sleep(RETRY_DELAY)
        self.get_logger().err(f"Failed to read encoder data after {MAX_RETRIES} retries.")
        return None, None

    def setSpeed(self, msg):
        self.speed = msg.data
        self.get_logger().info(f"Speed set to {self.speed}")

def main(args=None):
    rclpy.init(args=args)
    node = CanOpenNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
