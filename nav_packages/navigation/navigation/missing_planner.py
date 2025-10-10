import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math
import time

class ExecutionManager(Node):
    def __init__(self):
        super().__init__('missing_planner_node')
        
        # Create a publisher for '/pose2d_topic' of type Pose2D
        self.publisher = self.create_publisher(Pose2D, '/target_vessel_pose', 10)
        
        # Set a timer to publish messages at regular intervals
        self.timer = self.create_timer(1.0, self.publish_pose2d)  # Publish every second
        self.usv_current_pose = [-40.0, 0.0, 0.0] #for simple env
        self.target_vessel_pose = [70.0, 20.0, -1.0]
       
        self.get_logger().info("ExecutionManager initialized")
        self.counter = 0  # Example counter to simulate dynamic publishing

    def publish_pose2d(self):
        # Create and publish a Pose2D message
        pose_msg = Pose2D()
        pose_msg.x = self.target_vessel_pose[0]  # Example: sine wave for x
        pose_msg.y = self.target_vessel_pose[1]  # Example: cosine wave for y
        pose_msg.theta = self.target_vessel_pose[2]  # Example: theta in radians
        
        self.publisher.publish(pose_msg)
        self.get_logger().info(f"Published Pose2D: x={pose_msg.x}, y={pose_msg.y}, theta={pose_msg.theta}")
        
        self.counter += 0.1  # Increment counter for dynamic message generation
        

def main(args=None):
    rclpy.init(args=args)
    
    node = ExecutionManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
