#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from x3cator_msgs.msg import FourGasSensor

class FourGasSensorSubscriber(Node):
    def __init__(self):
        super().__init__('gas_sensor_subscriber')
        self.subscription = self.create_subscription(
            FourGasSensor,
            '/four_gas_sensor',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        self.get_logger().info("Gas sensor subscriber node started.")

    def listener_callback(self, msg):
        self.get_logger().info(
            f"Received sensor data - CO: {msg.co:.2f} ppm, H2S: {msg.h2s:.2f} ppm, "
            f"O2: {msg.o2:.2f} %, CH4: {msg.ch4:.2f} %LEL"
        )

def main(args=None):
    rclpy.init(args=args)
    node = FourGasSensorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down subscriber.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
