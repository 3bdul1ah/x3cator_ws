#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from x3cator_msgs.msg import FourGasSensor
import random
import time

class FakeGasSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_gas_sensor_publisher')
        
        self.publisher = self.create_publisher(FourGasSensor, '/four_gas_sensor', 10)
        
        self.timer = self.create_timer(1.0, self.publish_fake_data)  # 1 second period

    def publish_fake_data(self):
        msg = FourGasSensor()
        
        msg.co = random.uniform(0.0, 50.0)  
        msg.h2s = random.uniform(0.0, 10.0) 
        msg.o2 = random.uniform(18.0, 21.0) 
        msg.ch4 = random.uniform(0.0, 5.0)  
        
        self.get_logger().info(f"Publishing: CO: {msg.co:.2f}, H2S: {msg.h2s:.2f}, O2: {msg.o2:.2f}, CH4: {msg.ch4:.2f}")
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args) 
    node = FakeGasSensorPublisher() 
    
    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
