Simulation Setup
=================

Below are the commands that should be run in different terminals to load the USV simulation setup and necessary nodes. 

.. note:: before performing the steps below, please follow the installation steps explained in mbzirc repository. The link to that repository can be found  here `https://github.com/osrf/mbzirc <https://github.com/osrf/mbzirc>`_ .



.. warning:: Run ``source ~/mbzirc_ws/install/setup.bash`` in each new terminal before running the command. 

Load the world in Ignition. ::

    ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r simple_demo.sdf"

Spawn the USV with the Lidar sensor attached to the ``slot0``. and 3d camera attached to ``slot1`` ::

    ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv x:=-40 y:=0 z:=0.3 R:=0 P:=0 Y:=0 slot1:=mbzirc_planar_lidar slot2:=mbzirc_rgbd_camera 

.. note:: The command below is not needed if the usv is spawned with the above command, for custom sensor_msgs
          we need to call the ros-ign-bridge manually. 

Call *ros-ign-bridge* to bridge the lidar data from ign topic ``/world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan`` to ros2 topic ``/scan``. ::

    ros2 run ros_ign_bridge parameter_bridge /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /world/simple_demo/model/usv/model/sensor_0/link/sensor_link/sensor/lidar/scan:=/scan

Set the transformation between map and odom frame to math ``0 0 0``. ::

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom