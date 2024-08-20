import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber') # name of subscriber

        # info of topic we are subscribing to, data type, name of topic, 
        # how to process message, and how many messages can be stored in subscribers 
        # queue
        self.subscription = self.create_subscription(
            String,
            'basic_topic',
            self.listener_callback,
            10
        )
        self.subscription # prevents unused variable warning
    
    # function that records message when node intercepts it
    def listener_callback(self, msg):
        self.get_logger().info(f'I heard {msg.data}!')


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()