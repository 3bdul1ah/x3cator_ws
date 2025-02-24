#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Import the ROS2 message type (make sure this message type is built properly)
from x3cator_msgs.msg import FourGasSensor as FourGasSensorMsg
# Import your sensor reading class from your package
from gas_sensor.gas_reading import FourGasSensor

class FourGasSensorPublisher(Node):
    def __init__(self):
        super().__init__('gas_sensor_publisher')
        self.publisher = self.create_publisher(FourGasSensorMsg, '/four_gas_sensor', 10)
        self.sensor = FourGasSensor()
        # Timer calls publish_sensor_data every 0.2 seconds
        self.timer = self.create_timer(0.2, self.publish_sensor_data)
        self.get_logger().info("Gas sensor publisher node started.")

    def publish_sensor_data(self):
        try:
            sensor_data = self.sensor.read_sensor()
            msg = FourGasSensorMsg()
            msg.co = sensor_data.CO
            msg.h2s = sensor_data.H2S
            msg.o2 = sensor_data.O2
            msg.ch4 = sensor_data.CH4

            self.get_logger().info(
                f"Publishing: CO: {msg.co:.2f} ppm, H2S: {msg.h2s:.2f} ppm, "
                f"O2: {msg.o2:.2f} %, CH4: {msg.ch4:.2f} %LEL"
            )
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error reading sensor data: {e}")

    def destroy_node(self):
        self.sensor.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FourGasSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down publisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
