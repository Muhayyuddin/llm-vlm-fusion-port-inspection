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
    "; sleep 75 "
    "; echo 'spawining USV' "
    "; ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv x:=-10 y:=28 z:=0.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_naive_spinning_radar "  
    "; exec bash\"",
        # Open a new tab and display time and an error message
    "gnome-terminal --tab -- bash -c \" cd mbzirc_ws" 
    "; source install/setup.bash "
    "; echo 'waiting for lidar topic to call the ign ros bridge ... '  "
    "; sleep 100 "
    "; echo 'spawining USV' "
    "; ros2 run ros_ign_bridge parameter_bridge /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan:=/laser_scan "  
    "; exec bash\"",
     # Open a new tab and display time and the current user
    "gnome-terminal --tab -- bash -c \" cd mbzirc_ws "
    "; source install/setup.bash "  
    "; echo 'waiting for rviz...' "
    "; sleep 110 "
    "; rviz2 "  
    "; exec bash\"",
    # Open a new tab and display time and the current user
    "gnome-terminal --tab -- bash -c \" cd mbzirc_ws "
    "; source install/setup.bash "  
    "; echo 'waiting for the USV to spawn before running publisher to move the USV...' "
    "; sleep 120 "
    "; ros2 run py_pubsub talker "  
    "; exec bash\"",
   )

for c in commands:
    subprocess.run(shlex.split(c))
    time.sleep(2)
