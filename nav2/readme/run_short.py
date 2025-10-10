import subprocess
import shlex
import time


commands = (
    # Open a new tab and display time and message
    "gnome-terminal --tab -- bash -c \" cd mbzirc_ws"
    "; source install/setup.bash " 
    "; sleep 5 "
    "; ros2 launch mbzirc_ros competition_local.launch.py ign_args:='-v 4 -r simple_demo.sdf' "
    "; exec bash\"",
    # Open a new tab and display time and an error message
    "gnome-terminal --tab -- bash -c \" cd mbzirc_ws" 
    "; source install/setup.bash "
    "; echo 'waiting for Ignition to load the world model before spawning the USV... '  "
    "; sleep 5 "
    "; echo 'spawining USV' "
    "; ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv x:=-10 y:=28 z:=0.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_naive_spinning_radar "  
    "; exec bash\"",
        # Open a new tab and display time and an error message
    "gnome-terminal --tab -- bash -c \" cd mbzirc_ws" 
    "; source install/setup.bash "
    "; echo 'waiting for lidar topic to call the ign ros bridge ... '  "
    "; sleep 10 "
    "; echo 'spawining USV' "
    "; ros2 run ros_ign_bridge parameter_bridge /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan:=/laser_scan "  
    "; exec bash\"",
   )

for c in commands:
    subprocess.run(shlex.split(c))
    time.sleep(2)
