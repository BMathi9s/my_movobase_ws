#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class RectangularMecanumNode(Node):
    def __init__(self):
        super().__init__('rectangular_mecanum_node')

        # === Declare and retrieve parameters ===
        # Wheel radius (meters)
        self.declare_parameter('wheel_radius', 0.0762)  # e.g., 7.62 cm (3 in)
        # Robot dimension: distance between front and back wheels
        self.declare_parameter('robot_length', 0.50)    # total length (m)
        # Robot dimension: distance between left and right wheels
        self.declare_parameter('robot_width', 0.445)    # total width (m)

        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        # Half-length, half-width
        self.Lx = self.get_parameter('robot_length').get_parameter_value().double_value / 2.0
        self.Ly = self.get_parameter('robot_width').get_parameter_value().double_value / 2.0

        self.get_logger().info(
            f"RectangularMecanumNode started.\n"
            f"  wheel_radius={self.r:.4f} m\n"
            f"  robot_length={self.Lx*2:.4f} m  robot_width={self.Ly*2:.4f} m\n"
            f"  => Lx={self.Lx:.4f} m, Ly={self.Ly:.4f} m"
        )

        # === Subscribe to /cmd_vel (Twist) ===
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # === Publisher for wheel velocities (Float32MultiArray) ===
        self.wheel_cmd_pub = self.create_publisher(Float32MultiArray, '/movo_base/wheel_cmds', 10)

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert cmd_vel (vx, vy, wz) to four wheel velocities
        for a rectangular Mecanum layout.
        """
        vx = msg.linear.x  # m/s (forward/backward)
        vy = msg.angular.z   # m/s (left/right)
        wz = -msg.linear.y  # rad/s (yaw)




        # Using the rectangular Mecanum formula:
        #   w0 = (vx - vy - wz*(Lx+Ly)) / r  (front-left)
        #   w1 = (vx + vy + wz*(Lx+Ly)) / r  (front-right)
        #   w2 = (vx + vy - wz*(Lx+Ly)) / r  (back-right)
        #   w3 = (vx - vy + wz*(Lx+Ly)) / r  (back-left)
        
        w0 = (vx - vy + wz*(self.Lx + self.Ly)) / self.r
        w1 = (vx + vy  - wz*(self.Lx + self.Ly)) / self.r
        w2 = (vx - vy - wz*(self.Lx + self.Ly)) / self.r
        w3 = (vx + vy + wz*(self.Lx + self.Ly)) / self.r

        # Create and publish Float32MultiArray message
        wheel_cmds = Float32MultiArray()
        # motor_names = ["Front left", "Front right", "Back left", "Back right"]
        wheel_cmds.data = [-w0, w1, -w2, w3]
        self.wheel_cmd_pub.publish(wheel_cmds)

        # Debug log
        self.get_logger().info(
            f"cmd_vel -> vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f} => "
            f"FL={w0:.2f}, FR={w1:.2f}, BL={w2:.2f}, BR={w3:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RectangularMecanumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("RectangularMecanumNode interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
