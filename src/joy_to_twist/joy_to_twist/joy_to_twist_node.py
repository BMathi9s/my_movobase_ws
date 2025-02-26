# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Joy
# from geometry_msgs.msg import Twist


# class JoyToTwistNode(Node):
#     def __init__(self):
#         super().__init__('joy_to_twist_node')
#         self.get_logger().info("JoyToTwistNode started")

#         # Parameters for axis mapping
#         self.declare_parameter('axis_linear', 1)  # Default: left stick vertical
#         self.declare_parameter('axis_angular', 0)  # Default: left stick horizontal
#         self.declare_parameter('scale_linear', 1.0)
#         self.declare_parameter('scale_angular', 1.0)

#         self.axis_linear = self.get_parameter('axis_linear').value
#         self.axis_angular = self.get_parameter('axis_angular').value
#         self.scale_linear = self.get_parameter('scale_linear').value
#         self.scale_angular = self.get_parameter('scale_angular').value

#         # Subscribe to the /joy topic
#         self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

#         # Publish to the /cmd_vel topic
#         self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 100)

#     def joy_callback(self, msg):
#         twist = Twist()
#         # Map joystick axes to Twist
#         twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
#         twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular

#         # Publish the Twist message
#         self.twist_pub.publish(twist)

#         self.get_logger().info(f"Published Twist: linear.x = {twist.linear.x}, angular.z = {twist.angular.z}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = JoyToTwistNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('JoyToTwistNode interrupted.')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToTwistNode(Node):
    def __init__(self):
        super().__init__('joy_to_twist_node')
        self.get_logger().info("JoyToTwistNode started")

        # Parameters for axis mapping
        self.declare_parameter('axis_linear_x', 1)  # Default: left stick vertical (linear.x)
        self.declare_parameter('axis_linear_y', 0)  # Default: left stick horizontal (linear.y)
        self.declare_parameter('axis_angular', 2)   # Default: right stick horizontal (angular.z)
        self.declare_parameter('scale_linear', 1.0)
        self.declare_parameter('scale_angular', 1.0)

        self.axis_linear_x = self.get_parameter('axis_linear_x').value
        self.axis_linear_y = self.get_parameter('axis_linear_y').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        # Subscribe to the /joy topic
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Publish to the /cmd_vel topic
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 100)

    def joy_callback(self, msg):
        twist = Twist()
        # Map joystick axes to Twist
        twist.linear.x = msg.axes[self.axis_linear_x] * self.scale_linear
        twist.linear.y = msg.axes[self.axis_linear_y] * self.scale_linear
        twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular

        # Publish the Twist message
        self.twist_pub.publish(twist)

        self.get_logger().info(
            f"Published Twist: linear.x = {twist.linear.x}, linear.y = {twist.linear.y}, angular.z = {twist.angular.z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = JoyToTwistNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('JoyToTwistNode interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
