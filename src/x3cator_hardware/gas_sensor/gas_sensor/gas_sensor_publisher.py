#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from x3cator_msgs.msg import FourGasSensor
import serial
import ctypes

class FourGasSensorNode(Node):
    def __init__(self, port):
        super().__init__('four_gas_sensor_node') 
        self.sensor = serial.Serial(port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
        self.sensor.timeout = 0.5 
        
        self.sensor_pub = self.create_publisher(FourGasSensor, '/four_gas_sensor', 10)

    def spin(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg() 
        header.frame_id = 'four_gas_sensor'
        
        try:
            inBytes = [0xFF, 0x86]
            while self.sensor.read(1) != b'\xff':
                continue
            while self.sensor.read(1) != b'\x86':
                continue
            inBytes.extend(self.sensor.read(9))
            
            checksumByte = inBytes.pop()
            checksum = ctypes.c_ubyte(~((sum(inBytes)) % 256)).value
            if checksum == checksumByte:
                # Parse sensor data
                CO = int.from_bytes([inBytes[2], inBytes[3]], 'big', signed=False) * 1.00
                H2S = int.from_bytes([inBytes[4], inBytes[5]], 'big', signed=False) * 1.00
                O2 = int.from_bytes([inBytes[6], inBytes[7]], 'big', signed=False) * 0.1
                CH4 = int.from_bytes([inBytes[8], inBytes[9]], 'big', signed=False) * 1.00
                self.get_logger().info(f"CO: {CO:.2f}, H2S: {H2S:.2f}, O2: {O2:.2f}, CH4: {CH4:.2f}")
                
                msg = FourGasSensor()
                msg.header = header
                msg.co = max(CO, 0.0)  
                msg.h2s = max(H2S, 0.0) 
                msg.o2 = O2
                msg.ch4 = CH4 - 100.0  
                
                self.sensor_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Error reading from sensor: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = FourGasSensorNode(port='/dev/ttyUSB0') 
    
    try:
        while rclpy.ok():
            node.spin()  
            rclpy.spin_once(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() 
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
