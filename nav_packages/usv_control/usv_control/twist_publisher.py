#!/usr/bin/env python
'''
Using instances of the pypid Pid class to control yaw and velocity
'''
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import pi
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import csv
from datetime import datetime
import time

from sensor_msgs.msg import Imu

# BSB
from usv_pid.pypid import Pid

class PidControl(Node):
    def __init__(self,engaged=False,yaw_cntrl=True,vel_cntrl=True):
        super().__init__('pid_control')
        # Setup Yaw Pid
        self.engaged = engaged
        self.yaw_cntrl = yaw_cntrl
        self.vel_cntrl = vel_cntrl
        self.Kp=20.0
        self.Ki=0.001
        self.Kd=25.0
        self.ypid = Pid(self.Kp,self.Ki,self.Kd)
        
        self.ypid.set_setpoint(0.0)
        self.ypid.set_inputisangle(True,pi)
        self.ypid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.ypid.set_derivfilter(1,wc)

        self.vpid = Pid(40.0, 40.0, 0.01)
        self.vpid.set_setpoint(0.0)
        self.vpid.set_maxIout(1.0)
        self.vpid.set_derivfeedback(True)  # D term in feedback look
        fc = 20;  # cutoff freq in hz
        wc = fc*(2.0*pi)  # cutoff freq. in rad/s
        self.vpid.set_derivfilter(1,wc)
        
        # Initialize some bits as none - for now
        self.drivemsg = None
        self.publisher = None
        self.lasttime = None
        # For diagnosing/tuning PID
        self.vpubdebug = None
        self.ypubdebug = None
        
        self.left_cmd = Float64()
        self.right_cmd = Float64()
        # Type of yaw control
        self.yaw_type = 'yaw'

                # Setup publisher
        self.left_publisher = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.right_publisher = self.create_publisher(Float64,'/usv/right/thrust/cmd_thrust', 10)
        self.timer_period = 0.1  # seconds

        # Setup subscribers
        self.s1 = self.create_subscription(Odometry,'/usv/odom',self.odom_callback,10)
        self.s2 = self.create_subscription(Twist,"/cmd_vel",self.twist_callback,10)

        self.create_subscription(Imu,'/usv/imu/data',self.imu_callback,50)
        self.yaw = 0.0
        
        # Initialize CSV logging for error data
        self.start_time = time.time()
        self.csv_filename = f"{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_usv_pid_errors.csv"
        self.init_csv_file()
        
        self.get_logger().info(f"USV PID Controller initialized - saving error data to: {self.csv_filename}")

    def imu_callback(self,msg):

        euler = self.euler_from_quaternion(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        self.yaw = euler[2]

    def twist_callback(self,msg):
        print('Current twist is ', msg.angular.z)
        self.ypid.set_setpoint(msg.angular.z)
        self.vpid.set_setpoint(msg.linear.x)

    def course_callback(self,msg):
        self.ypid.set_setpoint(msg.yaw)
        self.vpid.set_setpoint(msg.speed)

    def odom_callback(self,msg):
        # Calculate time step
        now = self.get_clock().now()
        if self.lasttime is None:
            self.lasttime = now
            return
        dt = now.to_msg().sec-self.lasttime.to_msg().sec

        self.lasttime = now
        # Yaw Control
        if self.yaw_cntrl:
            #if self.yaw_type=='yaw_rate':
            #yaw_fdbk = msg.twist.twist.angular.z # measured rate (process var.)

            yout = self.ypid.execute(dt,self.yaw)
            torque = yout[0]
            yaw_error = yout[1] if len(yout) > 1 else 0.0  # Get error if available
            print('received torque   ', torque)
        else:
            torque = 0.0
            yaw_error = 0.0


        # Velocity control
        if self.vel_cntrl:
            dx = msg.twist.twist.linear.x
            vout = self.vpid.execute(dt,dx)
            thrust = vout[0]
            vel_error = vout[1] if len(vout) > 1 else 0.0  # Get error if available
        else:
            thrust = 0.0
            vel_error = 0.0

        # I believe drive messages are scaled to -1.0 to 1.0
        # Scale so that no one output saturates
        '''
        mag = abs(torque)+abs(thrust)
        if mag > 1.0:
            torque = torque/mag
            thrust = thrust/mag
        '''

        #rospy.loginfo('Torque: %.3f, Thrust: %.3f'%(torque,thrust))
        print('torque ', torque)
        print('thrust ',thrust)
        self.left_cmd.data = (-1.0*torque + thrust)
        self.right_cmd.data = (torque + thrust)
        # Only publish if engaged
        #if (self.engaged):
        self.left_publisher.publish(self.left_cmd)
        self.right_publisher.publish(self.right_cmd)
        
        # Save error data to CSV
        self.save_errors_to_csv(dt, yaw_error, vel_error, torque, thrust, 
                               self.ypid.get_setpoint() if hasattr(self.ypid, 'get_setpoint') else 0.0,
                               self.vpid.get_setpoint() if hasattr(self.vpid, 'get_setpoint') else 0.0,
                               self.yaw, dx if self.vel_cntrl else 0.0)

    def init_csv_file(self):
        """Initialize CSV file with headers"""
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "Relative Time (s)",
                "dt", 
                "Yaw Error", 
                "Velocity Error",
                "Torque Output", 
                "Thrust Output",
                "Yaw Setpoint",
                "Velocity Setpoint", 
                "Current Yaw",
                "Current Velocity",
                "Left Thrust Command",
                "Right Thrust Command"
            ])

    def save_errors_to_csv(self, dt, yaw_error, vel_error, torque, thrust, 
                          yaw_setpoint, vel_setpoint, current_yaw, current_vel):
        """Save control errors and commands to CSV"""
        current_time = time.time()
        relative_time = current_time - self.start_time
        
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                relative_time,
                dt,
                yaw_error,
                vel_error, 
                torque,
                thrust,
                yaw_setpoint,
                vel_setpoint,
                current_yaw,
                current_vel,
                self.left_cmd.data,
                self.right_cmd.data
            ])


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
        
        return roll_x, pitch_y, yaw_z # in radians    
        
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  # ROS Parameters

  # Create the node
  pid_control = PidControl(True,True,True)

  try:
    rclpy.spin(pid_control)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()

    
     