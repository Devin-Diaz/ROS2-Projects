import rclpy  # Import the ROS2 Python client library
from rclpy.node import Node  # Import the Node class, which is the base class for all ROS2 nodes
from sensor_msgs.msg import Image  # Import the Image message type from the sensor_msgs package
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS2 images and OpenCV images
import cv2  # Import OpenCV for image handling


class ImageSubscriber(Node):
    def __init__(self):
        """
        Initializes the ImageSubscriber node. This node subscribes to a topic to receive images,
        converts them to OpenCV format, and displays them using OpenCV.
        """
        super().__init__('image_subscriber')  # Initialize the node with the name 'image_subscriber'
        
        # Create a subscription to the 'image_topic' to receive messages of type Image
        # The listener_callback function will be called whenever a message is received
        self.subscription = self.create_subscription(
            Image,
            'image_topic',
            self.listener_callback,
            10
        )
        
        self.bridge = CvBridge()  # Instantiate CvBridge to handle conversion between ROS2 Image messages and OpenCV images


    def listener_callback(self, msg):
        """
        This function is called whenever an Image message is received on the subscribed topic.
        It converts the message to an OpenCV image and displays it.
        """
        self.get_logger().info('RECEIVING IMAGE')  # Log a message to indicate that an image is being received
        
        # Convert the ROS2 Image message to an OpenCV image (BGR format)
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the received image in a window named 'RECEIVED IMAGE'
        cv2.imshow('RECEIVED IMAGE', cv_img)
        cv2.waitKey(1)  # Wait for 1 millisecond to allow OpenCV to update the display window


def main(args=None):
    """
    The main function initializes the ROS2 Python client library, creates the ImageSubscriber node,
    and keeps it running until it is manually shut down.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    image_subscriber = ImageSubscriber()  # Create an instance of the ImageSubscriber node
    rclpy.spin(image_subscriber)  # Keep the node running, allowing it to process incoming events
    image_subscriber.destroy_node()  # Destroy the node when the application is stopped
    rclpy.shutdown()  # Shut down the ROS2 Python client library

if __name__ == '__main__':  # Check if this script is being run directly
    main()  # Run the main function
