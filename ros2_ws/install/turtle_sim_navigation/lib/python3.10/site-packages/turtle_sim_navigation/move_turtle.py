import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

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




