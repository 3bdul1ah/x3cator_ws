#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from x3cator_msgs.msg import GasSensorData
from datetime import datetime
import serial
import ctypes
import time

class GasSensorPublisher(Node):

    def __init__(self):
        super().__init__('gas_sensor_publisher')
        self.publisher_ = self.create_publisher(GasSensorData, 'gas_sensor_data', 10)
        self.timer = self.create_timer(1.0, self.publish_data)  # 1-second interval
        self.sensor = self.initialize_sensor()

    def initialize_sensor(self):
        """Initialize the sensor."""
        try:
            sensor = serial.Serial(
                port='/dev/ttyUSB1',
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
            return sensor
        except serial.SerialException as e:
            if 'Permission denied' in str(e):
                self.get_logger().error("Permission denied accessing /dev/ttyUSB1. Run with sudo.")
            raise

    def read_sensor(self):
        """Read and parse data from the sensor."""
        while self.sensor.read(1) != b'\xff':
            continue
        while self.sensor.read(1) != b'\x86':
            continue

        inBytes = [0xFF, 0x86]
        inBytes.extend(self.sensor.read(9))
        checksumByte = inBytes.pop()
        checksum = ctypes.c_ubyte(~((sum(inBytes)) % 256)).value

        if checksum == checksumByte:
            CO = int.from_bytes([inBytes[2], inBytes[3]], 'big') * 1.00
            H2S = int.from_bytes([inBytes[4], inBytes[5]], 'big') * 1.00
            O2 = int.from_bytes([inBytes[6], inBytes[7]], 'big') * 0.1
            CH4 = int.from_bytes([inBytes[8], inBytes[9]], 'big') * 1.00

            CO = max(0, CO)
            H2S = max(0, H2S)
            CH4 = CH4 - 100.0

            return GasSensorData(
                header=self.get_time_header(),
                CO=CO,
                H2S=H2S,
                O2=O2,
                CH4=CH4
            )
        else:
            raise ValueError("Checksum verification failed")

    def publish_data(self):
        try:
            sensor_data = self.read_sensor()
            self.publisher_.publish(sensor_data)
            self.get_logger().info(f"Published: CO={sensor_data.CO}, H2S={sensor_data.H2S}, O2={sensor_data.O2}, CH4={sensor_data.CH4}")
        except Exception as e:
            self.get_logger().error(f"Failed to read or publish data: {e}")
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = GasSensorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
