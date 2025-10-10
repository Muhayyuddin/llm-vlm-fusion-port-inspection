Important Tips & Tricks
=======================

Below are some usefull commands tips to work with ignition and ros2.

Set the static transformation between map and odom frame to math ``0 0 0``. ::

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

To view the transformation frames tree. ::

    ros2 run tf2_tools view_frames 

Call *ros-ign-bridge* to bridge the lidar data from ign topic ``/world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan`` to ros2 topic ``/scan``. ::

    ros2 run ros_ign_bridge parameter_bridge /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan:=/scan

Launching coast environment spawining the arm mounted USV and camera at slot0 ::

    ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast.sdf"
    ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0  arm:=mbzirc_oberon7_arm gripper:=mbzirc_oberon7_gripper slot0:=mbzirc_rgbd_camera
