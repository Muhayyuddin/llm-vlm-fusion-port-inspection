#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import select
import termios
import tty

class USVKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('usv_keyboard_teleop')
        
        # Publishers for thrust commands
        self.left_pub = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/usv/right/thrust/cmd_thrust', 10)
        
        # Thrust values
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        
        # Thrust increment per key press
        self.thrust_increment = 10.0
        self.max_thrust = 150.0
        self.min_thrust = -150.0
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_thrust)
        
        # Terminal settings for keyboard input
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("USV Keyboard Teleop Node Started")
        self.get_logger().info("Controls:")
        self.get_logger().info("  UP Arrow    - Increase forward thrust")
        self.get_logger().info("  DOWN Arrow  - Increase backward thrust")
        self.get_logger().info("  LEFT Arrow  - Turn left (differential thrust)")
        self.get_logger().info("  RIGHT Arrow - Turn right (differential thrust)")
        self.get_logger().info("  SPACE       - Stop all thrusters")
        self.get_logger().info("  q/Q         - Quit")
        
    def get_key(self):
        """Get a single keypress from terminal"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def process_key(self, key):
        """Process keyboard input and update thrust values"""
        if key == '\x1b':  # ESC sequence for arrow keys
            key2 = sys.stdin.read(1)
            key3 = sys.stdin.read(1)
            
            if key2 == '[':
                if key3 == 'A':  # UP Arrow
                    self.increase_forward_thrust()
                elif key3 == 'B':  # DOWN Arrow
                    self.increase_backward_thrust()
                elif key3 == 'D':  # LEFT Arrow
                    self.turn_left()
                elif key3 == 'C':  # RIGHT Arrow
                    self.turn_right()
                    
        elif key == ' ':  # SPACE
            self.stop_all()
        elif key.lower() == 'q':  # Quit
            return False
            
        return True
    
    def increase_forward_thrust(self):
        """Increase forward thrust for both thrusters"""
        self.left_thrust = min(self.left_thrust + self.thrust_increment, self.max_thrust)
        self.right_thrust = min(self.right_thrust + self.thrust_increment, self.max_thrust)
        self.get_logger().info(f"Forward thrust: Left={self.left_thrust:.2f}, Right={self.right_thrust:.2f}")
    
    def increase_backward_thrust(self):
        """Increase backward thrust for both thrusters"""
        self.left_thrust = max(self.left_thrust - self.thrust_increment, self.min_thrust)
        self.right_thrust = max(self.right_thrust - self.thrust_increment, self.min_thrust)
        self.get_logger().info(f"Backward thrust: Left={self.left_thrust:.2f}, Right={self.right_thrust:.2f}")
    
    def turn_left(self):
        """Turn left using differential thrust"""
        # Reduce left thrust, increase right thrust
        self.left_thrust = max(self.left_thrust - self.thrust_increment, self.min_thrust)
        self.right_thrust = min(self.right_thrust + self.thrust_increment, self.max_thrust)
        self.get_logger().info(f"Turn left: Left={self.left_thrust:.2f}, Right={self.right_thrust:.2f}")
    
    def turn_right(self):
        """Turn right using differential thrust"""
        # Increase left thrust, reduce right thrust
        self.left_thrust = min(self.left_thrust + self.thrust_increment, self.max_thrust)
        self.right_thrust = max(self.right_thrust - self.thrust_increment, self.min_thrust)
        self.get_logger().info(f"Turn right: Left={self.left_thrust:.2f}, Right={self.right_thrust:.2f}")
    
    def stop_all(self):
        """Stop all thrusters"""
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.get_logger().info("All thrusters stopped")
    
    def publish_thrust(self):
        """Publish thrust commands to the thrusters"""
        left_msg = Float64()
        right_msg = Float64()
        
        left_msg.data = self.left_thrust
        right_msg.data = self.right_thrust
        
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)
    
    def run(self):
        """Main run loop for keyboard input"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if not self.process_key(key):
                    break
                    
                # Process any pending ROS callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.stop_all()
            self.get_logger().info("USV Keyboard Teleop Node Stopped")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = USVKeyboardTeleop()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
