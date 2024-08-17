import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()
    
    def listener_callback(self, msg):
        self.get_logger().info('RECEIVING IMAGE')
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('RECEIVED IMAGE', cv_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()