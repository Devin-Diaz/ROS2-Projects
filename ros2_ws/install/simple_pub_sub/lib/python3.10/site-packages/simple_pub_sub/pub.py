import rclpy # standard ros library to bring life into the framework
from rclpy.node import Node # parent node object we inherit from when making custom node
from std_msgs.msg import String # used to provide metadata about timing and source of data

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher') # name of node
        # type of message publisher will send, name of topic publisher will send to, 
        # queue size before old messages get deleted 
        self.publisher = self.create_publisher(String, 'basic_topic', 10)
        timer_period = 0.5 # seconds

        # publisher will send message every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0 # counter for each message sent

    # Every time this function is called in our timer, we will write and publish a message
    def timer_callback(self):
        msg = String() # data type of our message
        msg.data = f'Hello ROS 2 World! [{self.i}]' # our message to the topic
        self.publisher.publish(msg) # action to actually send message
        self.get_logger().info(f'Published {msg.data} successfully') # notifies if publish went through 
        self.i += 1

def main(args=None):
    # prepares ros2 framework to begin working and establishes communications between nodes, topics, services...
    rclpy.init(args=args) 
    minimal_publisher = MinimalPublisher() # makes instance of our custom node
    rclpy.spin(minimal_publisher) # allows our programming to keep looping ensuring we are still communicating
    minimal_publisher.destroy_node() # safely shuts down node and frees up resources in our program
    rclpy.shutdown() # shuts down ros2 system


if __name__ == 'main':
    main()