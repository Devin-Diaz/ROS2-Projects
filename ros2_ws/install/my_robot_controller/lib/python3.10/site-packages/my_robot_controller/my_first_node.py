#!/usr/bin/env python3

# Import the core modules from the ROS 2 Python client library.
import rclpy
from rclpy.node import Node

# Define a class 'MyNode' that inherits from the 'Node' class.
# This class represents a single node in the ROS 2 system.
class MyNode(Node):
    def __init__(self):
        # Call the constructor of the 'Node' class with the name of this node.
        # This initializes the node and registers it with the ROS 2 system.
        super().__init__("first_node")

        # Initialize a counter variable. This will be used to keep track of how many times the timer callback is executed.
        self.counter = 0

        # Create a timer that triggers the 'timer_callback' method every 1.0 second.
        # The 'create_timer' method sets up the periodic execution of a function.
        self.create_timer(1.0, self.timer_callback)

    # Define the callback function that will be called by the timer.
    # This function will execute every time the timer triggers (once per second in this case).
    def timer_callback(self):
        # Log a message to the ROS 2 logging system. This message will be output to the console.
        # The message includes the value of the 'counter' variable.
        self.get_logger().info(f"Wasgud ROS2 {self.counter}")

        # Increment the counter by 1 each time the callback is executed.
        self.counter += 1

'''
The main function is the entry point of the script.
- It initializes the ROS 2 communication system.
- It creates an instance of 'MyNode' (which represents our ROS 2 node).
- It enters a loop that keeps the node active and responsive.
- Finally, it shuts down the ROS 2 communication system when the node is no longer needed.
'''
def main(args=None):
    # Initialize the ROS 2 system. This sets up the communication framework needed for nodes to interact.
    rclpy.init(args=args)

    # Create an instance of 'MyNode', which initializes our node with the functionality defined above.
    node = MyNode()

    # Enter a loop that keeps the node running, allowing it to process callbacks like the timer we set up.
    rclpy.spin(node)

    # Once the node is no longer needed (e.g., if the process is terminated), shut down the ROS 2 system cleanly.
    rclpy.shutdown()

# The following ensures that the 'main' function is called when this script is run from the command line.
# It allows the script to be executed as a standalone program.
if __name__ == '__main__':
    main()
