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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float64
import sys,tty,termios

class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

    def get():
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break
        if k=='\x1b[A':
                #print("up")
                return "Forward"
        elif k=='\x1b[B':
                #print("down")
                return "Back"
        elif k=='\x1b[C':
                #print("right")
                return "Right"
        elif k=='\x1b[D':
                #print("left")
                return "Left"
        elif k=='z':
                #print("left")
                return "Stop"
        else:
                print("not an arrow key!")
                return "Exit"

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.thrust_L = self.create_publisher(Float64, '/usv/left/thrust/cmd_thrust', 10)
        self.thrust_R = self.create_publisher(Float64, '/usv/right/thrust/cmd_thrust', 10)
        self.L_thrust_joint = self.create_publisher(Float64, '/usv/left/thrust/joint/cmd_pos', 10)
        self.R_thrust_joint = self.create_publisher(Float64, '/usv/right/thrust/joint/cmd_pos', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.getkey = _Getch

    def timer_callback(self):
        thrust_L = Float64()
        thrust_R = Float64()

        turn = Float64()

        k=self.getkey.get()
        print(k)

        if k == "Forward":
            thrust_L.data = 150.0
            thrust_R.data = 150.0
            turn.data = 0.0
        elif k == "Back":
            thrust_L.data = -150.0
            thrust_R.data = -150.0
            turn.data = 0.0
        elif k == "Right":
            thrust_L.data = 0.0
            thrust_R.data = 100.0
            turn.data = 0.5
        elif k == "Left":
            thrust_L.data = 100.0
            thrust_R.data = 0.0
            turn.data = -0.5
        elif k == "Stop":
            thrust_L.data = 0.0
            thrust_R.data = 0.0
            turn.data = -0.5
        if k == "Exit":
            rclpy.shutdown()

        self.thrust_L.publish(thrust_L)
        self.thrust_R.publish(thrust_R)
        #self.L_thrust_joint.publish(turn)
        #self.R_thrust_joint.publish(turn)

        #self.get_logger().info('Thrust : " %s " \n turn : " %s "' % str(thrust.data), str(turn.data))

    def getKey(settings):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def saveTerminalSettings():
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
