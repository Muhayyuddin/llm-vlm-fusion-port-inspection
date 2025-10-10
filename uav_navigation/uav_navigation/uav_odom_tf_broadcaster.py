#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import csv
from datetime import datetime
import math

class UAVTFBroadcaster(Node):

    def __init__(self):
        """
        Class constructor to set up the UAV TF broadcaster node
        """
        super().__init__('uav_tf_broadcaster')

        # Subscription and publisher setup
        self.pose_sub = self.create_subscription(
            TFMessage, 
            '/quadrotor_1/pose_static', 
            self.tf_callback, 
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/uav/odom', 10)
        
        # TF broadcaster
        self.br = TransformBroadcaster(self)
        
        # Initialize odometry message
        self.odom = Odometry()
        
        # CSV logging setup
        self.start_time = None
        self.csv_filename = f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_uav_position_data.csv"
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Relative Time (s)", "Position X", "Position Y", "Position Z", "Roll", "Pitch", "Yaw"])

        self.get_logger().info('UAV TF Broadcaster node initialized')
        self.get_logger().info(f'Subscribing to: /quadrotor_1/pose_static')
        self.get_logger().info(f'Publishing odometry to: /uav/odom')
        self.get_logger().info(f'CSV logging to: {self.csv_filename}')

    def tf_callback(self, msg):
        """
        Callback function to process TF message and find quadrotor_1 frame
        """
        # Find the quadrotor_1 transform in the message
        quadrotor_transform = None
        for transform in msg.transforms:
            if transform.child_frame_id == 'quadrotor_1':
                quadrotor_transform = transform
                break
        
        if quadrotor_transform is None:
            self.get_logger().warn('quadrotor_1 frame not found in TF message')
            return

        # Log the transform data
        self.get_logger().debug(f'Received transform for quadrotor_1: '
                               f'pos({quadrotor_transform.transform.translation.x:.2f}, '
                               f'{quadrotor_transform.transform.translation.y:.2f}, '
                               f'{quadrotor_transform.transform.translation.z:.2f})')

        # Create and publish TF
        self.publish_tf(quadrotor_transform)
        
        # Create and publish odometry
        self.publish_odometry(quadrotor_transform)
        
        # Log to CSV
        self.log_to_csv(quadrotor_transform)

    def publish_tf(self, transform):
        """
        Publish TF transform from odom to uav frame
        """
        t = TransformStamped()
        t.header.stamp = transform.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'uav'
        
        # Copy position
        t.transform.translation.x = transform.transform.translation.x
        t.transform.translation.y = transform.transform.translation.y
        t.transform.translation.z = transform.transform.translation.z
        
        # Copy orientation
        t.transform.rotation.x = transform.transform.rotation.x
        t.transform.rotation.y = transform.transform.rotation.y
        t.transform.rotation.z = transform.transform.rotation.z
        t.transform.rotation.w = transform.transform.rotation.w
        
        # Broadcast the transform
        self.br.sendTransform(t)

    def publish_odometry(self, transform):
        """
        Publish odometry message
        """
        self.odom.header.stamp = transform.header.stamp
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'uav'
        
        # Set position
        self.odom.pose.pose.position.x = transform.transform.translation.x
        self.odom.pose.pose.position.y = transform.transform.translation.y
        self.odom.pose.pose.position.z = transform.transform.translation.z
        
        # Set orientation
        self.odom.pose.pose.orientation.x = transform.transform.rotation.x
        self.odom.pose.pose.orientation.y = transform.transform.rotation.y
        self.odom.pose.pose.orientation.z = transform.transform.rotation.z
        self.odom.pose.pose.orientation.w = transform.transform.rotation.w
        
        # Set covariance (optional - you can tune these values)
        self.odom.pose.covariance[0] = 0.1   # x
        self.odom.pose.covariance[7] = 0.1   # y
        self.odom.pose.covariance[14] = 0.1  # z
        self.odom.pose.covariance[21] = 0.1  # roll
        self.odom.pose.covariance[28] = 0.1  # pitch
        self.odom.pose.covariance[35] = 0.1  # yaw
        
        # Velocity is not available from static pose, set to zero
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = 0.0
        
        # Publish odometry
        self.odom_pub.publish(self.odom)

    def log_to_csv(self, transform):
        """
        Log position and orientation data to CSV file
        """
        if self.start_time is None:
            self.start_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        
        current_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        relative_time = current_time - self.start_time
        
        # Convert quaternion to euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )
        
        # Write to CSV
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                relative_time,
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                roll,
                pitch,
                yaw
            ])

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    uav_tf_broadcaster = UAVTFBroadcaster()
    
    try:
        uav_tf_broadcaster.get_logger().info('Starting UAV TF broadcaster...')
        rclpy.spin(uav_tf_broadcaster)
    except KeyboardInterrupt:
        uav_tf_broadcaster.get_logger().info('UAV TF broadcaster interrupted by user')
    except Exception as e:
        uav_tf_broadcaster.get_logger().error(f'UAV TF broadcaster crashed: {e}')
    finally:
        uav_tf_broadcaster.get_logger().info('Shutting down UAV TF broadcaster')
        uav_tf_broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()