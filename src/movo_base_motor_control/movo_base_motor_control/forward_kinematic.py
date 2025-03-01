#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math

class ForwardKinematicNode(Node):
    def __init__(self):
        super().__init__('forward_kinematic_node')
        
        # Declare parameters (matching the inverse kinematics node)
        self.declare_parameter('wheel_radius', 0.076)  # in meters
        self.declare_parameter('robot_length', 0.50)     # total length (m)
        self.declare_parameter('robot_width', 0.445)       # total width (m)
        
        # Retrieve parameters
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.robot_length = self.get_parameter('robot_length').get_parameter_value().double_value
        self.robot_width = self.get_parameter('robot_width').get_parameter_value().double_value
        self.Lx = self.robot_length / 2.0
        self.Ly = self.robot_width / 2.0

        self.get_logger().info(
            f"ForwardKinematicNode started with parameters:\n"
            f"  wheel_radius: {self.r:.4f} m\n"
            f"  robot_length: {self.robot_length:.4f} m, robot_width: {self.robot_width:.4f} m\n"
            f"  Lx: {self.Lx:.4f} m, Ly: {self.Ly:.4f} m"
        )
        
        # Robot pose (for odometry integration)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Subscriber: wheel states (expects 4 values: [FL, FR, BL, BR])
        self.create_subscription(
            Float32MultiArray,
            '/movo_base/wheel_states',
            self.wheel_states_callback,
            10
        )
        
        # Publisher: odometry message
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
    
    def wheel_states_callback(self, msg: Float32MultiArray):
        # Log the raw received message
        self.get_logger().info(f"Received wheel_states: {msg.data}")
        
        if len(msg.data) != 4:
            self.get_logger().error("Received wheel_states does not contain 4 elements.")
            return
        
        # The inverse kinematics node publishes:
        #   FL = -w0, FR = w1, BL = -w2, BR = w3
        # Therefore, recover the "w" values:
        FL_meas = msg.data[0]
        FR_meas = msg.data[1]
        BL_meas = msg.data[2]
        BR_meas = msg.data[3]
        self.get_logger().info(
        f"Raw wheel velocities: FL_meas={msg.data[0]:.6f}, FR_meas={msg.data[1]:.6f}, BL_meas={msg.data[2]:.6f}, BR_meas={msg.data[3]:.6f}"
        )

        
        # Un-invert the left side wheels
        w0 = -FL_meas   # front-left
        w1 = FR_meas    # front-right
        w2 = -BL_meas   # back-left
        w3 = BR_meas    # back-right
        
        # Log the un-inverted wheel speeds
        # self.get_logger().info(
        #     f"Computed wheel speeds: w0={w0:.6f}, w1={w1:.6f}, w2={w2:.6f}, w3={w3:.6f}"
        # )
        
        # Forward kinematics equations derived from the inverse mapping:
        # vx = (r/4) * (w0 + w1 + w2 + w3)
        # vy = (r/4) * (-w0 + w1 - w2 + w3)
        # ω  = [ (w3 - w1) + (w0 - w2) ] / (4*(Lx+Ly))
        vx = (self.r / 4.0) * (w0 + w1 + w2 + w3)
        vy = (self.r / 4.0) * (-w0 + w1 - w2 + w3)
        wz = ((w3 - w1) + (w0 - w2)) / (4.0 * (self.Lx + self.Ly))
        
        # Log the computed velocities
        self.get_logger().info(
            f"Computed velocities: vx={vx:.6f} m/s, vy={vy:.6f} m/s, wz={wz:.6f} rad/s"
        )
        
        # Compute time difference for integration
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # seconds
        self.get_logger().info(f"Time delta (dt): {dt:.6f} s")
        self.last_time = current_time
        
        # Integrate velocities to update pose (using simple Euler integration)
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.x += delta_x
        self.y += delta_y
        self.theta += wz * dt
        # Normalize theta between -pi and pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Build the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        # Convert yaw (theta) to quaternion (assuming roll=pitch=0)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Twist (velocity)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        
        # Publish odometry
        self.odom_pub.publish(odom_msg)
        
        self.get_logger().info(
            f"Published odometry: x={self.x:.6f} m, y={self.y:.6f} m, theta={self.theta:.6f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
