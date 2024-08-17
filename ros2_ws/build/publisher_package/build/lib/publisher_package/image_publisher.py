import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self) -> None:
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.img = cv2.imread('/home/devin/Documents/Github-Projects/ROS2_Projects/ros2_ws/src/ros.png')

    def timer_callback(self):  # Corrected function name
        if self.img is not None:  # Corrected condition check
            msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('PUBLISHING IMAGE')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':  # Corrected main function condition
    main()
