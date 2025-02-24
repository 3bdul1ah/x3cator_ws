import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
import numpy as np

class GasConcentrationMapper(Node):
    def __init__(self):
        super().__init__('gas_concentration_mapper')

        # Subscriber to SLAM map
        self.slam_map_sub = self.create_subscription(
            OccupancyGrid,
            '/slam/map',
            self.slam_map_callback,
            10
        )

        # Publisher for the gas map
        self.gas_map_pub = self.create_publisher(
            OccupancyGrid,
            '/gas/map',
            10
        )

        self.gas_data = None
        self.slam_map = None

        # Timer to simulate gas sensor reading
        self.create_timer(1.0, self.read_gas_sensor_data)

    def read_gas_sensor_data(self):
        # Simulate reading from a virtual gas sensor
        # Here, you should subscribe to an actual gas sensor topic in your robot
        gas_concentration = 0.5  # For example, assume 50% gas concentration
        self.gas_data = gas_concentration

        # If we have both SLAM map and gas data, update the gas map
        if self.slam_map is not None and self.gas_data is not None:
            self.update_gas_map()

    def slam_map_callback(self, msg):
        """Callback to update the SLAM map."""
        self.slam_map = msg
        self.get_logger().info('Received SLAM map')

    def update_gas_map(self):
        """Update gas map based on SLAM map and gas concentration."""
        if self.slam_map is None or self.gas_data is None:
            return

        # Create a new gas map based on the SLAM map
        gas_map = np.array(self.slam_map.data).reshape(
            self.slam_map.info.height, self.slam_map.info.width)

        # Set gas concentration in unoccupied spaces (value 0 means free space)
        for i in range(gas_map.shape[0]):
            for j in range(gas_map.shape[1]):
                if gas_map[i, j] == 0:  # If free space in the map
                    gas_map[i, j] = int(self.gas_data * 100)

        # Prepare the gas map message
        updated_gas_map = OccupancyGrid()
        updated_gas_map.header.stamp = self.get_clock().now().to_msg()
        updated_gas_map.header.frame_id = self.slam_map.header.frame_id
        updated_gas_map.info = self.slam_map.info
        updated_gas_map.data = gas_map.flatten().tolist()

        # Publish the gas map
        self.gas_map_pub.publish(updated_gas_map)
        self.get_logger().info('Published merged gas map')

def main(args=None):
    rclpy.init(args=args)
    node = GasConcentrationMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
