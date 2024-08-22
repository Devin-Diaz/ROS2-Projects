import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, TeleportAbsolute
import math

class TurtleObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('turtle_obstacle_avoidance')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.obstacle_names = ['obstacle1', 'obstacle2', 'obstacle3']
        self.obstacle_positions = [(5.0, 5.0), (8.0, 8.0), (2.0, 7.0)]
        self.current_pose = None

        # Teleport turtle to a safe starting position away from obstacles
        self.teleport_turtle(5.5, 1.0, math.pi / 2)

        # Spawn the obstacle turtles
        for name, position in zip(self.obstacle_names, self.obstacle_positions):
            self.spawn_turtle(position[0], position[1], 0.0, name)

        self.timer = self.create_timer(0.1, self.move_turtle)

    def teleport_turtle(self, x, y, theta):
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport service...')

        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = teleport_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Teleported turtle to ({x}, {y})')
        else:
            self.get_logger().error('Failed to teleport turtle')

    def spawn_turtle(self, x, y, theta, name):
        spawn_client = self.create_client(Spawn, '/spawn')
        while not spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for spawn service to create {name}...')

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned {name} at ({x}, {y})')
        else:
            self.get_logger().error(f'Failed to spawn {name}')

    def pose_callback(self, msg):
        self.current_pose = msg

    def move_turtle(self):
        if self.current_pose is None:
            return

        msg = Twist()
        safe_distance = 2.0  # Minimum distance to maintain from obstacles

        # Check proximity to each obstacle
        for obstacle_name, obstacle_position in zip(self.obstacle_names, self.obstacle_positions):
            distance = self.calculate_distance(
                self.current_pose.x,
                self.current_pose.y,
                obstacle_position[0],
                obstacle_position[1]
            )

            if distance < safe_distance:
                # If too close to an obstacle, turn away
                msg.linear.x = 0.0
                msg.angular.z = 1.57
                msg.linear.x = 2.0
                msg.angular.z = -0.5

                break
        else:
            # If no obstacle is too close, move forward
            msg.linear.x = 2.0
            msg.angular.z = 0.0

        self.publisher.publish(msg)

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    

def main(args=None):
    rclpy.init(args=args)
    turtle_obstacle_avoidance = TurtleObstacleAvoidance()
    rclpy.spin(turtle_obstacle_avoidance)
    turtle_obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
