#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        
        # Create publisher for target vessel pose
        self.publisher = self.create_publisher(String, '/target_vessel_pose', 10)
        
        # Publish the target pose immediately
        self.publish_target_pose(-1400, 15, 0.0)
        
    def publish_target_pose(self, x, y, z):
        """Publish target pose in the format expected by navigator"""
        msg = String()
        # Format the message as expected by the navigator's target_pose_callback
        location_data = {"x": x, "y": y, "z": z}
        msg.data = f"move_to Location: {location_data}"
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Published target pose: x={x}, y={y}, z={z}")
        self.get_logger().info(f"Message: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    publisher = TargetPosePublisher()
    
    # Keep the node alive for a moment to ensure message is published
    rclpy.spin_once(publisher, timeout_sec=1.0)
    
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
