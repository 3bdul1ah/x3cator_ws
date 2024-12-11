#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from x3cator_msgs.msg import GasSensorData

class GasSensorSubscriber(Node):

    def __init__(self):
        super().__init__('gas_sensor_subscriber')
        self.subscription = self.create_subscription(
            GasSensorData,
            'gas_sensor_data',
            self.listener_callback,
            10
        )
        self.subscription 

    def listener_callback(self, msg):
        self.get_logger().info(f"Received sensor data: CO={msg.CO}, H2S={msg.H2S}, O2={msg.O2}, CH4={msg.CH4}")

def main(args=None):
    rclpy.init(args=args)
    node = GasSensorSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
