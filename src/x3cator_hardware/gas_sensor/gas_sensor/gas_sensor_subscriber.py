#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from x3cator_msgs.msg import FourGasSensor

class FourGasSensorSubscriber(Node):
    def __init__(self):
        super().__init__('four_gas_sensor_subscriber')
        self.create_subscription(FourGasSensor, '/four_gas_sensor', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received CO: {msg.co}, H2S: {msg.h2s}, O2: {msg.o2}, CH4: {msg.ch4}")
        
def main(args=None):
    rclpy.init(args=args)  
    node = FourGasSensorSubscriber() 
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  
        rclpy.shutdown() 

if __name__ == '__main__':
    main()
