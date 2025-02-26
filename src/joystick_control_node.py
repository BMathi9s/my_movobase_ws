import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__("joystick_control_node")

        # Publishers for wheel velocity commands
        self.front_right_pub = self.create_publisher(Float32, "/movo_base/wheel_cmds/front_right", 10)
        self.front_left_pub = self.create_publisher(Float32, "/movo_base/wheel_cmds/front_left", 10)
        self.back_right_pub = self.create_publisher(Float32, "/movo_base/wheel_cmds/back_right", 10)
        self.back_left_pub = self.create_publisher(Float32, "/movo_base/wheel_cmds/back_left", 10)

        # Subscriber for joystick input
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Robot-specific parameters
        self.max_velocity = 2.0  # Maximum wheel velocity (m/s)

        self.get_logger().info("JoystickControlNode initialized. Listening to joystick input...")

    def joy_callback(self, msg: Joy):
        # Extract joystick inputs
        left_stick_x = msg.axes[0]  # Left joystick horizontal
        left_stick_y = msg.axes[1]  # Left joystick vertical
        right_trigger = (1 - msg.axes[5]) / 2.0  # Right trigger (scaled from 0 to 1)

        # Calculate robot velocities
        linear_velocity = right_trigger * self.max_velocity  # Scale trigger to max velocity
        angular_velocity = left_stick_x * self.max_velocity  # Scale horizontal axis to max velocity

        # Convert robot velocities to individual wheel velocities
        front_right_vel, front_left_vel, back_right_vel, back_left_vel = self.calculate_wheel_velocities(
            linear_velocity, angular_velocity
        )

        # Publish wheel velocities
        self.front_right_pub.publish(Float32(data=front_right_vel))
        self.front_left_pub.publish(Float32(data=front_left_vel))
        self.back_right_pub.publish(Float32(data=back_right_vel))
        self.back_left_pub.publish(Float32(data=back_left_vel))

        self.get_logger().info(
            f"Linear: {linear_velocity:.2f}, Angular: {angular_velocity:.2f} | "
            f"FR: {front_right_vel:.2f}, FL: {front_left_vel:.2f}, BR: {back_right_vel:.2f}, BL: {back_left_vel:.2f}"
        )

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        # Mecanum wheel velocity calculation for differential motion
        # Velocities are derived based on a simple kinematic model
        front_right_vel = linear_velocity - angular_velocity
        front_left_vel = linear_velocity + angular_velocity
        back_right_vel = linear_velocity - angular_velocity
        back_left_vel = linear_velocity + angular_velocity

        # Clamp velocities to the max limit
        front_right_vel = max(min(front_right_vel, self.max_velocity), -self.max_velocity)
        front_left_vel = max(min(front_left_vel, self.max_velocity), -self.max_velocity)
        back_right_vel = max(min(back_right_vel, self.max_velocity), -self.max_velocity)
        back_left_vel = max(min(back_left_vel, self.max_velocity), -self.max_velocity)

        return front_right_vel, front_left_vel, back_right_vel, back_left_vel


def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt detected. Stopping...")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
