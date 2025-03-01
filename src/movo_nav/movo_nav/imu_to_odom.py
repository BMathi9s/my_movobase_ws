#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

class ImuToOdom(Node):
    def __init__(self):
        super().__init__('imu_to_odom')
        
        # Subscribers:
        # Subscribing to the full IMU message to get linear acceleration and angular velocity.
        self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        # Subscribing to the repurposed MagneticField message carrying roll, pitch, yaw values.
        self.create_subscription(MagneticField, 'imu_data_rpy', self.rpy_callback, 10)
        
        # Publisher:
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Initialize variables for integration
        self.last_time = self.get_clock().now()
        self.x = 0.0  # Position in world frame (x)
        self.y = 0.0  # Position in world frame (y)
        self.vx = 0.0  # Velocity in world frame (x)
        self.vy = 0.0  # Velocity in world frame (y)
        
        # Latest yaw from the IMU (in radians)
        self.yaw = 0.0

    def rpy_callback(self, msg: MagneticField):
        # In this repurposed topic, x, y, and z correspond to roll, pitch, yaw respectively.
        # For a 2D odom we only need yaw.
        self.yaw = msg.magnetic_field.z
        # You may log or check the value:
        # self.get_logger().info(f"Updated yaw: {self.yaw:.3f}")

    def imu_callback(self, msg: Imu):
        # Get the current time and compute dt for integration.
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # Use the linear acceleration from the IMU.
        # Note: Make sure the acceleration data is gravity-compensated.
        a_x = msg.linear_acceleration.x
        a_y = msg.linear_acceleration.y

        # To integrate in the world frame, transform the acceleration from the robot (IMU) frame.
        # Assuming the IMU is aligned with the robot and the robot moves in the horizontal plane.
        world_a_x = a_x * math.cos(self.yaw) - a_y * math.sin(self.yaw)
        world_a_y = a_x * math.sin(self.yaw) + a_y * math.cos(self.yaw)

        # Integrate acceleration to update velocity.
        self.vx += world_a_x * dt
        self.vy += world_a_y * dt

        # Integrate velocity to update position.
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Create an Odometry message.
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Set the position.
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Convert yaw to a quaternion.
        # For 2D, roll and pitch are assumed to be zero.
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        # Set the twist (velocities). Using the integrated velocity and the angular velocity directly from the IMU.
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        # Using the z angular velocity from the IMU message.
        odom.twist.twist.angular.z = msg.angular_velocity.z

        # (Optionally, set covariance matrices if you have an estimate of your sensor noise.)
        # For example, you could set diagonal values if you know your uncertainties.
        # Here we leave them as default (all zeros).

        # Publish the odometry message.
        self.odom_pub.publish(odom)
        # For debugging, you can log the computed pose:
        # self.get_logger().info(f"Odom: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
