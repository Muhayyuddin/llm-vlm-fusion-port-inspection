#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from .pid_controller import PIDController
from .guidance_controller import GuidanceController

class UAVNavigationNode(Node):
    def __init__(self):
        super().__init__('uav_navigation_node')
        
        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/quadrotor_1/cmd_vel', 10)
        self.imu_sub = self.create_subscription(Imu, '/quadrotor_1/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/uav/odom', self.odom_callback, 10)

        # Controllers
        self.pid_controller = PIDController()
        self.guidance_controller = GuidanceController()
        
        # Navigation parameters
        self.target_waypoints = [
            Point(x=0.0, y=0.0, z=15.0),
            Point(x=0.0, y=35.0, z=15.0),
            Point(x=70.0, y=35.0, z=15.0),
            Point(x=70.0, y=70.0, z=15.0),
            Point(x=0.0, y=0.0, z=15.0),
            Point(x=0.0, y=0.0, z=0.0)

        ]
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.5
        
        # Current state
        self.current_position = None
        self.current_orientation = None
        
        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('UAV Navigation Node initialized')

    def imu_callback(self, msg):
        """Process IMU data for orientation"""
        self.current_orientation = msg.orientation

    def odom_callback(self, msg):
        """Process odometry data for position"""
        self.current_position = msg.pose.pose.position
        self.get_logger().info(f'Current position updated: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}, z={self.current_position.z:.2f}')


    def control_loop(self):
        """Main control loop"""
        # Print current waypoint index and total waypoints
        self.get_logger().info(f'Control Loop - Waypoint Index: {self.current_waypoint_index}/{len(self.target_waypoints)}')
        
        if self.current_waypoint_index < len(self.target_waypoints):
            target = self.target_waypoints[self.current_waypoint_index]
            
            # Print target waypoint values
            self.get_logger().info(f'Target waypoint: x={target.x:.2f}, y={target.y:.2f}, z={target.z:.2f}')
            
            if self.current_position is None:
                self.get_logger().warn('No position data available yet')
                return
            
            # Print current position values
            self.get_logger().info(f'Current position: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}, z={self.current_position.z:.2f}')
            
            # Print position difference
            dx = target.x - self.current_position.x
            dy = target.y - self.current_position.y
            dz = target.z - self.current_position.z
            self.get_logger().info(f'Position difference: dx={dx:.2f}, dy={dy:.2f}, dz={dz:.2f}')
            
            # Compute desired velocity
            desired_velocity = self.guidance_controller.compute_target_velocity(
                target, self.current_position, self.current_orientation
            )
            
            # Print desired velocity values
            self.get_logger().info(f'Desired velocity: vx={desired_velocity.linear.x:.2f}, vy={desired_velocity.linear.y:.2f}, vz={desired_velocity.linear.z:.2f}')
            
            # Apply PID control
            control_output = self.pid_controller.update(desired_velocity, self.current_position)
            
            # Print control output values
            self.get_logger().info(f'Control output: x={control_output.get("linear", {}).get("x", 0.0):.2f}, '
                                 f'y={control_output.get("linear", {}).get("y", 0.0):.2f}, '
                                 f'z={control_output.get("linear", {}).get("z", 0.0):.2f}, '
                                 f'yaw={control_output.get("angular", {}).get("z", 0.0):.2f}')
            
            # Create and publish control command
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = control_output.get('linear', {}).get('x', 0.0)
            cmd_vel_msg.linear.y = control_output.get('linear', {}).get('y', 0.0)
            cmd_vel_msg.linear.z = control_output.get('linear', {}).get('z', 0.0)
            cmd_vel_msg.angular.z = control_output.get('angular', {}).get('z', 0.0)
            
            # Print final command being sent
            self.get_logger().info(f'Publishing cmd_vel: linear=({cmd_vel_msg.linear.x:.2f}, {cmd_vel_msg.linear.y:.2f}, {cmd_vel_msg.linear.z:.2f}), '
                                 f'angular.z={cmd_vel_msg.angular.z:.2f}')
            
            self.cmd_vel_pub.publish(cmd_vel_msg)

            # Check if waypoint is reached
            if self.is_waypoint_reached(target):
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached!')
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index >= len(self.target_waypoints):
                    self.get_logger().info('All waypoints completed!')
                    stop_msg = Twist()
                    self.cmd_vel_pub.publish(stop_msg)
        else:
            self.get_logger().info('All waypoints completed - hovering')

    def is_waypoint_reached(self, target):
        """Check if waypoint is reached"""
        if self.current_position is None:
            return False
            
        distance = self.calculate_distance(self.current_position, target)
        self.get_logger().info(f'Distance to waypoint: {distance:.2f}m')
        return distance < self.waypoint_threshold

    def calculate_distance(self, pos1, pos2):
        """Calculate 3D distance"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

def main(args=None):
    rclpy.init(args=args)
    node = UAVNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()