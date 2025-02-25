import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.capture_frame)  # 10 FPS
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open default camera

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera!")
            exit(1)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(image_msg)
        else:
            self.get_logger().warn("Failed to capture frame!")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

