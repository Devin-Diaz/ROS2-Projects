import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

'''
In this client node script, I'm noticing a similar pattern for interacting with services in ROS 2. The TurtleMoverClient class also extends the Node class, making it a ROS 2 node named 'turtle_mover_client'.

Service Client Setup: We create a client that communicates with the 'set_direction' service, which expects SetBool requests. The node waits until the service is available, ensuring it can send requests.

Request Sending Function: The send_request method sends either True or False to the service.

True instructs the service to move the turtle forward.
False makes the turtle rotate.
We use rclpy.spin_until_future_complete to wait for the service response, ensuring the request is handled before proceeding.
Main Function Flow:

First, the node sends a True request, moving the turtle forward.
A short delay (time.sleep(2)) is added to let the turtle move before sending the next request.
Then, it sends a False request to rotate the turtle.
Finally, the node is properly shut down after the requests are made.
'''

class TurtleMoverClient(Node):
    def __init__(self):
        super().__init__('turtle_mover_client')
        self.client = self.create_client(SetBool, 'set_direction')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetBool.Request()
    
    def send_request(self, move_forward: bool):
        self.req.data = move_forward
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    turtle_mover_client = TurtleMoverClient()
    response = turtle_mover_client.send_request(True)
    print(f'Response: {response.message}')

    time.sleep(2)

    response = turtle_mover_client.send_request(False)
    print(f'Response: {response.message}')

    turtle_mover_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




