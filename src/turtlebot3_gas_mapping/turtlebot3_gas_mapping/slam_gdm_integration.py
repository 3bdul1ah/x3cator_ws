import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

class SLAMGDMIntegration(Node):
    def __init__(self):
        super().__init__('slam_gdm_integration')
        self.slam_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.slam_callback,
            10
        )
        self.gas_subscriber = self.create_subscription(
            Float32,
            'gas_sensor',
            self.gas_callback,
            10
        )
        self.merged_map_publisher = self.create_publisher(
            OccupancyGrid,
            '/merged_map',
            10
        )
        self.slam_map = None
        self.gas_concentration = 0.0
        self.get_logger().info('SLAM-GDM Integration Node Started')

    def slam_callback(self, msg):
        self.slam_map = msg
        self.merge_maps()

    def gas_callback(self, msg):
        self.gas_concentration = msg.data
        self.merge_maps()

    def merge_maps(self):
        if self.slam_map is None:
            return

        # Create a copy of the SLAM map for modification
        merged_map = OccupancyGrid()
        merged_map.header = self.slam_map.header
        merged_map.info = self.slam_map.info
        merged_map.data = list(self.slam_map.data)

        # Replace unoccupied cells with gas concentration
        for i in range(len(merged_map.data)):
            if merged_map.data[i] == 0:  # Unoccupied cell
                merged_map.data[i] = int(self.gas_concentration)  # Gas concentration as an integer

        # Publish merged map
        self.merged_map_publisher.publish(merged_map)
        self.get_logger().info('Published Merged Map')


def main(args=None):
    rclpy.init(args=args)
    node = SLAMGDMIntegration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
