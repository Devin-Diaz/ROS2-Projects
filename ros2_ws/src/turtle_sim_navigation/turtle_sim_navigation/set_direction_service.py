import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # used to describe velocity commands for mobile robots
from std_srvs.srv import SetBool 

'''
In this script, I'm continuing to see patterns in how nodes are constructed in ROS 2. The DirectionService class extends the Node class, making it a ROS 2 node named 'direction_service'.

Publisher Setup: We create a publisher that will send velocity commands (Twist messages) to the topic /turtle1/cmd_vel with a queue size of 10. This means our node can send messages to control the turtle's movement.

Service Setup: The service is created using create_service, with the service type SetBool. This service allows other nodes to send True or False requests to the 'set_direction' service. The requests determine whether the turtle should move forward or rotate.

Callback Function: The core logic is in set_direction_callback. Here, we check the incoming request:

If True, the turtle moves forward (linear.x = 2.5).
If False, the turtle rotates (angular.z = 1.0).
After determining the movement, we publish the Twist message and return a success response.
'''

class DirectionService(Node):
    def __init__(self):
        super().__init__('direction_service')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Provides the name of the service that other nodes can request to, 
        # type of requests it'll process, in this case with SetBool we have T/F 
        # type requests (on/off). Lastly callback functions to give functionality
        # based off the request we receive
        self.srv = self.create_service(SetBool, 'set_direction', self.set_direction_callback)
    
    def set_direction_callback(self, request, response):
        msg = Twist()
        if request.data:
            msg.linear.x = 2.50
            msg.angular.z = 0.0
            response.message = "moving forward!"
        else:
            msg.linear.x = 0.0
            msg.angular.z = 1.0
            response.message = 'rotating!'
        
        self.publisher.publish(msg)
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    direction_service = DirectionService()
    rclpy.spin(direction_service)
    direction_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


