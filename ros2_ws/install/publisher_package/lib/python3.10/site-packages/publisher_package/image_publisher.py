import rclpy  # Import the ROS2 Python client library
from rclpy.node import Node  # Import the Node class, which is the base class for all ROS2 nodes
from sensor_msgs.msg import Image  # Import the Image message type from the sensor_msgs package
from cv_bridge import CvBridge  # Import CvBridge to convert between ROS2 images and OpenCV images
import cv2  # Import OpenCV for image handling

class ImagePublisher(Node):
    def __init__(self) -> None:
        """
        Initializes the ImagePublisher node. This node reads an image from the disk and publishes it
        to a ROS2 topic at a fixed interval.
        """
        super().__init__('image_publisher')  # Initialize the node with the name 'image_publisher'
        
        # Create a publisher that publishes messages of type Image on the topic 'image_topic'
        # The queue size is set to 10, which limits the number of queued messages before dropping old ones
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        
        timer_period = 1.0  # Define the interval (in seconds) between publications
        self.timer = self.create_timer(timer_period, self.timer_callback)  # Create a timer that triggers the callback function at the specified interval
        
        self.bridge = CvBridge()  # Instantiate CvBridge to handle conversion between ROS2 Image messages and OpenCV images
        
        # Read the image from the specified path. Ensure the path is correct and the image exists
        self.img = cv2.imread('/home/devin/Documents/Github-Projects/ROS2_Projects/ros2_ws/src/ros.png')

    def timer_callback(self):
        """
        This function is called periodically by the timer. It converts the loaded image to a ROS2 Image message
        and publishes it to the topic.
        """
        if self.img is not None:  # Check if the image was successfully loaded
            # Convert the OpenCV image (BGR format) to a ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(self.img, encoding='bgr8')
            
            # Publish the Image message to the topic
            self.publisher_.publish(msg)
            
            # Log a message to indicate that the image was published
            self.get_logger().info('PUBLISHING IMAGE')

def main(args=None):
    """
    The main function initializes the ROS2 Python client library, creates the ImagePublisher node,
    and keeps it running until it is manually shut down.
    """
    rclpy.init(args=args)  # Initialize the ROS2 Python client library
    image_publisher = ImagePublisher()  # Create an instance of the ImagePublisher node
    rclpy.spin(image_publisher)  # Keep the node running, allowing it to process incoming events
    image_publisher.destroy_node()  # Destroy the node when the application is stopped
    rclpy.shutdown()  # Shut down the ROS2 Python client library

if __name__ == '__main__':  # Check if this script is being run directly
    main()  # Run the main function
