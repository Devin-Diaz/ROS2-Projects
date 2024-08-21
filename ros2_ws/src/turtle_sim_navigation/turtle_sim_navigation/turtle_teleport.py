import rclpy
from turtlesim.srv import TeleportAbsolute, SetPen

def teleport_turtle(node, x, y, theta):
    """
    Teleports the turtle to a specific position in the TurtleSim environment without drawing a line.

    Parameters:
    node (Node): The ROS 2 node instance that calls this function.
    x (float): The x-coordinate to which the turtle should be teleported.
    y (float): The y-coordinate to which the turtle should be teleported.
    theta (float): The orientation of the turtle in radians. This controls the direction
                   the turtle is facing after teleportation.

    This function first lifts the turtle's pen to prevent drawing while teleporting,
    then teleports the turtle to the specified coordinates, and finally lowers the pen
    to resume drawing.
    """

    # Create a service client for the 'set_pen' service to control the pen
    set_pen_client = node.create_client(SetPen, '/turtle1/set_pen')
    
    # Wait until the 'set_pen' service is available
    while not set_pen_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for set_pen service...')

    # Lift the pen to avoid drawing while teleporting
    pen_request = SetPen.Request()
    pen_request.r = 255  # Red color component (not used in this case)
    pen_request.g = 255  # Green color component (not used in this case)
    pen_request.b = 255  # Blue color component (not used in this case)
    pen_request.width = 0  # Pen width (not relevant when pen is off)
    pen_request.off = 1  # Turn the pen off (lifted)
    set_pen_client.call_async(pen_request)

    # Create a service client for the 'teleport_absolute' service to teleport the turtle
    client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
    
    # Wait until the 'teleport_absolute' service is available
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for teleport service...')

    # Create and send the teleportation request
    request = TeleportAbsolute.Request()
    request.x = x  # Set the x-coordinate for teleportation
    request.y = y  # Set the y-coordinate for teleportation
    request.theta = theta  # Set the orientation (in radians) for teleportation
    client.call_async(request)

    # Lower the pen after teleporting to allow drawing
    pen_request.off = 0  # Turn the pen on (lowered)
    set_pen_client.call_async(pen_request)
