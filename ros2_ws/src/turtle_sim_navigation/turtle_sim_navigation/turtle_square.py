import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

'''
Recall, the rule of thumb when making a publisher node, especially with
turtlesim, we typically use Twist since this defines the movement of the 
turtle, and the channel of the turtlesim env is /turtle1/cmd_vel

With publisher nodes, we are constantly sending messages to a channel since
ain't no way a robot works off of one message, thus we create a timer field 
that contains, how often we send a message to this channel and of course the 
logic that actually produces this messsage which is function. This function
is also typically timer_callback.

More in depth with Twist, in our callback function, we make an instance of this,
and utilize it's movement functions, linear and angular. These properties have their
own sub-properties, x, y, z. For now since we are working in a 2-D env, the only
relevant directions are linear.x (forwards/backwards), and angular.z (rotational).
'''

class TurtleSquare(Node):
    def __init__(self):
        super().__init__('turtle_square')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.step_counter = 0
        self.state = 0
        self.counter = 0

    def timer_callback(self):
        msg = Twist()
        if self.state == 0:
            msg.linear.x = 1.5
            msg.angular.z = 0.0
            self.step_counter += 1
            if self.step_counter >= 2:
                self.state = 1
                self.step_counter = 0
                self.counter += 1

        elif self.state == 1:
            msg.linear.x = 0.0
            msg.angular.z = 1.57
            self.step_counter += 1
            if self.step_counter >= 1:
                self.state = 0
                self.step_counter = 0

        if self.counter >= 4:
            self.publisher.publish(msg)
            self.timer.cancel()
            return

        self.publisher.publish(msg)
                    
def main(args=None):
    rclpy.init(args=None)
    turtle_square = TurtleSquare()
    rclpy.spin(turtle_square)
    turtle_square.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

