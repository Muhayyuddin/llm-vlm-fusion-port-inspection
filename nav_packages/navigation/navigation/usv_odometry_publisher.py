#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import csv
from datetime import datetime

class USVOdometryPublisher(Node):
    """
    USV Odometry Publisher Node
    
    Subscribes to /usv/pose_static and publishes odometry to /usv/odom
    Also broadcasts TF transform from odom -> usv frame
    """

    def __init__(self):
        super().__init__('usv_odometry_publisher')

        # Get use_sim_time parameter (already declared by ROS 2)
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f"Using simulation time: {use_sim_time}")

        # Initialize variables
        self.current_position = None
        self.current_orientation = None

        # Subscription and publisher setup
        self.pose_subscription = self.create_subscription(
            TFMessage, 
            '/usv/pose_static', 
            self.pose_callback, 
            10
        )
        
        self.odom_publisher = self.create_publisher(Odometry, '/usv/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'usv'

        # Initialize CSV logging
        self.start_time = None
        self.csv_filename = f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_usv_position_data.csv"
        self.init_csv_file()

        self.get_logger().info("USV Odometry Publisher initialized")
        self.get_logger().info("Subscribing to /usv/pose_static")
        self.get_logger().info("Publishing odometry to /usv/odom")
        self.get_logger().info("Broadcasting TF: odom -> usv")
        self.get_logger().info(f"Saving position data to: {self.csv_filename}")

    def pose_callback(self, msg):
        """Process TF message from pose_static topic"""
        usv_transform = None
        
        # Find the USV transform in the TF message
        for transform in msg.transforms:
            if transform.child_frame_id == 'usv':
                usv_transform = transform
                break
        
        if usv_transform is None:
            self.get_logger().warn("No USV transform found in pose_static message")
            return

        # Update current position and orientation
        self.current_position = usv_transform.transform.translation
        self.current_orientation = usv_transform.transform.rotation
        
        # Log first position received
        if not hasattr(self, '_first_pos_logged'):
            self.get_logger().info(
                f"✅ First USV position received: "
                f"({self.current_position.x:.2f}, {self.current_position.y:.2f}, {self.current_position.z:.2f})"
            )
            self._first_pos_logged = True

        # Publish TF transform and odometry
        self.publish_tf_and_odometry(usv_transform)

    def publish_tf_and_odometry(self, transform):
        """Publish TF transform and odometry message"""
        # Use the ORIGINAL timestamp from simulation instead of current time
        # This is critical for SLAM synchronization
        original_timestamp = transform.header.stamp
        
        # Publish TF transform with original simulation timestamp
        tf_msg = TransformStamped()
        tf_msg.header.stamp = original_timestamp  # Keep original simulation time!
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'usv'
        tf_msg.transform = transform.transform
        self.tf_broadcaster.sendTransform(tf_msg)
        
        # Publish odometry with original simulation timestamp
        self.odom_msg.header.stamp = original_timestamp  # Keep original simulation time!
        self.odom_msg.pose.pose.position.x = transform.transform.translation.x
        self.odom_msg.pose.pose.position.y = transform.transform.translation.y
        self.odom_msg.pose.pose.position.z = 0.0  # Keep Z at 0 for surface vessel
        self.odom_msg.pose.pose.orientation = transform.transform.rotation
        
        # Set covariance (diagonal matrix with reasonable values)
        self.odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  # y
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  # z
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,  # roll
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,  # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1   # yaw
        ]
        
        self.odom_publisher.publish(self.odom_msg)
        
        # Save to CSV
        self.save_position_to_csv(
            self.odom_msg.pose.pose.position.x,
            self.odom_msg.pose.pose.position.y,
            self.odom_msg.pose.pose.position.z,
            original_timestamp,
            transform.transform.rotation
        )

    def init_csv_file(self):
        """Initialize CSV file with headers"""
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "Relative Time (s)", 
                "Position X", 
                "Position Y", 
                "Position Z", 
                "Roll (deg)", 
                "Pitch (deg)", 
                "Yaw (deg)",
                "Quaternion X",
                "Quaternion Y", 
                "Quaternion Z",
                "Quaternion W"
            ])

    def save_position_to_csv(self, x, y, z, timestamp, orientation):
        """Save position and orientation data to CSV file"""
        if self.start_time is None:
            # Convert ROS timestamp to seconds
            if hasattr(timestamp, 'nanoseconds'):
                self.start_time = timestamp.nanoseconds * 1e-9
            else:
                # Handle builtin_interfaces.msg.Time
                self.start_time = timestamp.sec + timestamp.nanosec * 1e-9

        # Convert ROS timestamp to seconds
        if hasattr(timestamp, 'nanoseconds'):
            current_time = timestamp.nanoseconds * 1e-9
        else:
            # Handle builtin_interfaces.msg.Time
            current_time = timestamp.sec + timestamp.nanosec * 1e-9
            
        relative_time = current_time - self.start_time

        # Calculate euler angles from quaternion
        roll, pitch, yaw = self.euler_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        # Save data to CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                relative_time, 
                x, y, z, 
                math.degrees(roll), 
                math.degrees(pitch), 
                math.degrees(yaw),
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            ])

    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    
    try:
        usv_odometry_publisher = USVOdometryPublisher()
        rclpy.spin(usv_odometry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'usv_odometry_publisher' in locals():
            usv_odometry_publisher.get_logger().info("USV Odometry Publisher shutting down")
            usv_odometry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
