# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from math import atan2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
import math
from numpy import sqrt
import numpy as np
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
np.set_printoptions(precision=3, suppress=True)  # neat printing
# Removed transforms3d imports - using NumPy operations instead
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from ast import literal_eval
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from random import seed



class USVNavigation(Node):

    def __init__(self):
        super().__init__('usv_navigation')
 
        #self.target_vessel_pose = [-1400, 20, -1.0]
        self.usv_current_pose = [-40.0, 0.0, 0.0] #for simple env
        self.usv_goal = [0.0, 0.0, 0.0]
        #self.usv_current_pose = [-1350.0, 60.5, 0.0]
        self.target_vessel_pose = [10.0, -20.0, -1.0]
        
        self.target_vessel = TransformStamped()
        self.goal_pose = TransformStamped()
        self._tf_publisher = StaticTransformBroadcaster(self)
        self._tf_publisher2 = StaticTransformBroadcaster(self)
        self.goal_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.make_transforms()
        seed(1)
        self.compute_goalpose()
        self.twist = Twist()
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        self.direction = 0
        self.obstacle_avoidance_mode = False
        self.publish_twist = self.create_publisher(Twist,"cmd_vel",  10)
        
        self.create_subscription(Imu,'/usv/imu/data',self.imu_callback,50)
        self.create_subscription(String,'/target_vessel_pose',self.target_pose_callback,50)
        self.create_subscription(LaserScan,'/usv/slot0/scan',self.laser_callback,50)
        self.create_subscription(Odometry, '/usv/odom', self.odom_callback,50)
        self.reach_subgoal_publisher = self.create_publisher(String, "/reach_subgoal", 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.yaw = 0.0
        self.speed = 0.0
        self.target_yaw = 0.0
        self.heading = False
        self.surge = False
        self.dist_threshold = 1.0 # meters
        self.yaw_threshold = 0.2 # radian
        self.angle = 0.0
        self.dacceleration_distance = 20
        self.turn_left = False
        self.trun_right = False
        self.obstacle_on_leftside = False
        self.obstacle_on_rightside = False
        self.enable_collision_checker = True
        self.distance_to_obstacle = 15
        self.publish_reach_subgoal()

    def target_pose_callback(self, msg):
        try:
            input_string = msg.data
            self.get_logger().info(f"Received: {input_string}")

            location_list = None  # Initialize location_list to handle cases when "move_to" is not in the input

            if "move_to" in input_string:
                # Extract the location from the string
                location_string = input_string.split("Location: ")[1]
                location = literal_eval(location_string)
                location_list = [float(location["x"]), float(location["y"]), float(location["z"])]
                self.get_logger().info(f"Parsed move_to location with float values: {location_list}")

                # Update the target vessel pose
                self.target_vessel_pose[0] = location_list[0]
                self.target_vessel_pose[1] = location_list[1]
                self.target_vessel_pose[2] = location_list[2]

                # Update the goal pose
                goal = self.compute_goalpose()
                self.usv_goal[0] = goal[0]
                self.usv_goal[1] = goal[1]
                self.usv_goal[2] = self.target_vessel_pose[2]

            elif "record_data" in input_string:
                # Activate the camera flag
                self.activate_camera = True
                self.get_logger().info(f"Action: record_data, activate_camera: {self.activate_camera}")

        except Exception as e:
            self.get_logger().error(f"Error processing input: {e}")
 

    def imu_callback(self,msg):
        # Get quaternion from Imu message
        euler = self.euler_from_quaternion(msg.orientation.x,
                                           msg.orientation.y,
                                           msg.orientation.z,
                                           msg.orientation.w)
        self.yaw = euler[2]

    def odom_callback(self, msg):
        self.usv_current_pose[0] = msg.pose.pose.position.x
        self.usv_current_pose[1] = msg.pose.pose.position.y
        #print(msg)

    def timer_callback(self):

        # if not self.obstacle_avoidance_mode :

        self.dist_to_goal = sqrt((self.usv_goal[0] - self.usv_current_pose[0])**2 + 
                                (self.usv_goal[1] - self.usv_current_pose[1])**2)

        if self.dist_to_goal < self.dist_threshold :
            print('USV is already near to the target, enable visual servoing ')
            self.twist.angular.z = 0.0
            self.twist.linear.x = 0.0
            self.publish_twist.publish(self.twist)
            return
        else:
            self.angle = self.get_target_yaw()
            # print('yaw: ', self.yaw)
            # print('position: ', self.usv_current_pose)
            print('Distance to goal is ', self.dist_to_goal)
            self.twist.angular.z = self.angle
            self.twist.linear.x = 0.0
            print("current yaw", abs(abs(self.yaw)))
            print("Desired yaw", abs(self.angle))

            if(abs(abs(self.yaw)-abs(self.angle))<0.1):
                self.heading = True
                print('heading is set start moving')
            else:
                self.heading = False

            if self.dist_to_goal<=20.0:
                self.distance_to_obstacle = 5.0

            if self.heading :
                if self.dist_to_goal <= self.dacceleration_distance:
                    self.twist.linear.x = float(min(((self.dist_to_goal/10.0)*2),2))
                    print('approaching goal distance and velocity are ', self.dist_to_goal, self.twist.linear.x)
                    if self.dist_to_goal <= (self.dacceleration_distance / 2)+10:
                        if self.dist_to_goal < 2:
                            self.publish_reach_subgoal()
                    self.distance_to_obstacle = 5.0
                else:    
                    self.twist.linear.x = 2.0
                    self.distance_to_obstacle = 15.0
            print('publishing twist..')

            self.publish_twist.publish(self.twist)
        # else:     
        #     self.twist.angular.z = self.angle
        #     self.twist.linear.x = self.speed
        #     self.publish_twist.publish(self.twist)

        # self.enable_collision_checker = True
    def publish_reach_subgoal(self):
        """Publish a message to /reach_subgoal."""
        message = String()
        message.data = "Trigger next goal"
        self.reach_subgoal_publisher.publish(message)
        self.get_logger().info("Published subgoal reached message to /reach_subgoal.")



    def get_target_yaw(self):

        self.Y =  self.usv_current_pose[1] - self.usv_goal[1] 
        self.X =  self.usv_current_pose[0] - self.usv_goal[0] 
        self.target_yaw =atan2(self.Y,self.X)
        angle = 0.0
        
        if self.target_yaw < 0 :
            angle = math.pi + self.target_yaw
        else:
            angle = self.target_yaw - math.pi
  
        return angle 

    def laser_callback(self, msg):

        if self.enable_collision_checker :
            obstacle_infront = False
            #USV front 
            print(len(msg.ranges))
            for index in range(300, 440, 10):
                if msg.ranges[index] < self.distance_to_obstacle:
                    self.obstacle_avoidance_mode = True

                    if index > 290 and index < 370:
                        self.angle = self.yaw + 1.5
                        self.turn_left = True
                        print('obstacle infront turn left')

                    else:
                        self.angle = self.yaw - 1.5
                        self.trun_right = True
                        print('obstacle infront turn right')


                    self.speed = 0.0
                    obstacle_infront = True
                    break

            if not obstacle_infront and self.turn_left:
                print('checking obstacle on rightside')
                self.obstacle_on_rightside = False
                for index in range(0, 300, 10):
                    print('obstacel distance at right ', msg.ranges[index])
                    if msg.ranges[index] < self.distance_to_obstacle:
                        print('Front is clear moving streight')
                        self.obstacle_avoidance_mode = True
                        self.angle = self.yaw
                        self.speed = 2.0
                        self.obstacle_on_rightside = True
                        break
                if not self.obstacle_on_rightside:
                    self.turn_left = False
                    self.obstacle_avoidance_mode = False
                    self.enable_collision_checker = False


            if not obstacle_infront and self.trun_right:
                print('checking obstacle on rightside')
                self.obstacle_on_leftside = False
                for index in range(440, 710, 10):
                    print('obstacel distance at right ', msg.ranges[index])
                    if msg.ranges[index] < self.distance_to_obstacle:
                        if index > 630 and index < 680:
                            continue
                        else:
                            print('Front is clear moving streight')
                            self.obstacle_avoidance_mode = True
                            self.angle = self.yaw
                            self.speed = 2.0
                            self.obstacle_on_leftside = True
                            break
                if not self.obstacle_on_leftside:
                    self.trun_right = False
                    self.obstacle_avoidance_mode = False
                    self.enable_collision_checker = False

        self.enable_collision_checker = False

    def compute_goalpose(self):
        """
        Compute goal pose using NumPy operations instead of transforms3d
        """
        # Target vessel position and rotation
        target_x = self.target_vessel_pose[0]
        target_y = self.target_vessel_pose[1]
        target_yaw = self.target_vessel_pose[2]
        
        # Create 2D rotation matrix for target yaw
        cos_yaw = np.cos(target_yaw)
        sin_yaw = np.sin(target_yaw)
        
        # Goal offset in target vessel's local frame (11 meters behind)
        local_offset_x = 0.0
        local_offset_y = -11.0
        
        # Transform the offset to global frame
        global_offset_x = cos_yaw * local_offset_x - sin_yaw * local_offset_y
        global_offset_y = sin_yaw * local_offset_x + cos_yaw * local_offset_y
        
        # Calculate final goal position
        goal_x = target_x + global_offset_x
        goal_y = target_y + global_offset_y
        
        goal = [goal_x, goal_y]
        print("goal values", goal_x, goal_y)
        
        return goal

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z 

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)

    usv_navigation = USVNavigation()

    rclpy.spin(usv_navigation)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    usv_navigation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


