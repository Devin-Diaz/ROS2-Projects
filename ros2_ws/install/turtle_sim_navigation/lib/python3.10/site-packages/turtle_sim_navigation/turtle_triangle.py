import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .turtle_teleport import teleport_turtle

class TurtleTriangle(Node):
    def __init__(self):
        super().__init__('turtle_triangle')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer = 1.0
        self.timer = self.create_timer(timer, self.timer_callback)
        self.counter = 0
        self.steps = 0
        self.state = 0

        teleport_turtle(self, 2.0, 5.5, 0.0)

    def timer_callback(self):
        msg = Twist()

        if self.state == 0:
            msg.linear.x = 1.5
            msg.angular.z = 0.0
            self.steps += 1
            if self.steps >= 2:
                self.state = 1
                self.steps = 0
                self.counter += 1
        elif self.state == 1:
            msg.angular.z = 2.094
            msg.linear.x = 0.0
            self.steps += 1
            if self.steps >= 1:
                self.state = 0
                self.steps = 0
        
        if self.counter >= 3:
            self.publisher.publish(msg)
            self.timer.cancel()
            return
    
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_triangle = TurtleTriangle()
    rclpy.spin(turtle_triangle)
    turtle_triangle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
