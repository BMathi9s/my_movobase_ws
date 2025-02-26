#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros
from laser_geometry import LaserProjection

class DualScanMerger(Node):
    def __init__(self):
        super().__init__('dual_scan_merger')
        # Declare parameters
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('merged_scan_topic', '/merged_scan')
        self.declare_parameter('scan_topics', ['/lidar_front/scan', '/lidar_back/scan'])
        
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.merged_scan_topic = self.get_parameter('merged_scan_topic').get_parameter_value().string_value
        scan_topics = self.get_parameter('scan_topics').get_parameter_value().string_array_value
        self.front_scan_topic = scan_topics[0]
        self.rear_scan_topic = scan_topics[1]

        # Subscribers for each scan
        self.front_scan = None
        self.rear_scan = None
        self.create_subscription(LaserScan, self.front_scan_topic, self.front_scan_callback, 10)
        self.create_subscription(LaserScan, self.rear_scan_topic, self.rear_scan_callback, 10)
        
        # Publisher for merged scan
        self.publisher_ = self.create_publisher(LaserScan, self.merged_scan_topic, 10)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Laser projection helper (from laser_geometry package)
        self.laser_projector = LaserProjection()
        
        # Timer to attempt merging scans periodically
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def front_scan_callback(self, msg):
        self.front_scan = msg

    def rear_scan_callback(self, msg):
        self.rear_scan = msg

    def timer_callback(self):
        if self.front_scan is None or self.rear_scan is None:
            return

        try:
            # Lookup transforms to the desired output frame
            transform_front = self.tf_buffer.lookup_transform(self.output_frame,
                                                              self.front_scan.header.frame_id,
                                                              rclpy.time.Time())
            transform_rear = self.tf_buffer.lookup_transform(self.output_frame,
                                                             self.rear_scan.header.frame_id,
                                                             rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn("TF lookup failed: " + str(e))
            return

        # For simplicity, we perform a basic merge:
        # We assume both scans have the same angular resolution.
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = self.front_scan.angle_increment  # assume both scans use the same resolution
        num_bins = int(round((angle_max - angle_min) / angle_increment))
        merged_ranges = [float('inf')] * num_bins

        def add_scan(scan, transform):
            angle = scan.angle_min
            for r in scan.ranges:
                # Skip invalid measurements
                if math.isinf(r) or math.isnan(r):
                    angle += scan.angle_increment
                    continue
                # Compute the point in the sensor frame
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                # Apply a simple 2D transform using the translation and yaw (extracted from the quaternion)
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                yaw = self.quaternion_to_yaw(transform.transform.rotation)
                x_out = math.cos(yaw) * x - math.sin(yaw) * y + tx
                y_out = math.sin(yaw) * x + math.cos(yaw) * y + ty
                r_out = math.hypot(x_out, y_out)
                theta = math.atan2(y_out, x_out)
                bin_index = int(round((theta - angle_min) / angle_increment))
                if 0 <= bin_index < num_bins:
                    merged_ranges[bin_index] = min(merged_ranges[bin_index], r_out)
                angle += scan.angle_increment

        add_scan(self.front_scan, transform_front)
        add_scan(self.rear_scan, transform_rear)

        # Create a merged LaserScan message
        merged_scan = LaserScan()
        merged_scan.header.stamp = self.get_clock().now().to_msg()
        merged_scan.header.frame_id = self.output_frame
        merged_scan.angle_min = angle_min
        merged_scan.angle_max = angle_max
        merged_scan.angle_increment = angle_increment
        merged_scan.time_increment = self.front_scan.time_increment
        merged_scan.scan_time = self.front_scan.scan_time
        merged_scan.range_min = self.front_scan.range_min
        merged_scan.range_max = self.front_scan.range_max
        merged_scan.ranges = merged_ranges

        self.publisher_.publish(merged_scan)

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw (2D)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

def main(args=None):
    rclpy.init(args=args)
    node = DualScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
