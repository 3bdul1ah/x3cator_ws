import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import random

class VirtualGasSensor(Node):
    def __init__(self):
        super().__init__('virtual_gas_sensor')
        self.publisher_ = self.create_publisher(Float32, 'gas_sensor', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 Hz frequency
        self.get_logger().info('Virtual Gas Sensor Node Started')

    def timer_callback(self):
        # Simulate gas concentration
        gas_concentration = random.uniform(0.0, 100.0)
        msg = Float32()
        msg.data = gas_concentration
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published gas concentration: {gas_concentration:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = VirtualGasSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
