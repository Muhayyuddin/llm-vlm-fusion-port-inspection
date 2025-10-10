Simulation Tools
=================
Below is the list of third party tools (ros2 packages) that are used for usv navigation.

Slam-tool-box
-------------
Slam-tool-box is used to build the map of the environment. Inorder to build the map following nodes should be running

    * Simulation should be running in ignition and usv spawned alog with lidar sensor. 
    * Transformation between odom to usv frame should be broadcast
    * Transformation between map to odom should be define.
    * Launch slam-tool-box. 

.. note:: Build the workspace by running ``IGNITION_VERSION=fortress colcon build --merge-install``. 


below is the list of the launch files that is will launch the above states nodes::

    ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r simple_demo.sdf"
    ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=simple_demo model:=usv x:=-40 y:=0 z:=0.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_planar_lidar slot1:=mbzirc_rgbd_camera 
    ros2 launch usv_description usv.launch.py 
    ros2 run custom_tf_broadcaster publish_odometry_and_tf 
    ros2 launch ros2_mapping online_async_launch.py

.. note:: Run the above states commands in different terminals. whenever opens the new terminal dont forget to run ``source ~/mbzirc_ws/install/setup.bash``. 

Once the map is completed, run the following command to save the map. ::

    ros2 run nav2_map_server map_sarver_cli -f my_map

Navigation2 (Nav2)
------------------
Navigation2 is used for autonomous navigation. Once the map is saved. the following nodes are required to load the map.

    * map_server
    * nav2_lifecycle_manager

The following command will launch these nodes with required settings. ::

    ros2 launch ros2_mapping map_bringsup.launch.py 

The following command will launch the nav2  for path path planning along with the required configurations. ::

    ros2 launch nav2_bringup navigation_launch.py 'params_file:=/home/muhayy/mbzirc_ws/src/usv_description/config/nav2_params.yaml'

Robot Localization
------------------


Cartographer
-------------

Cartographer is used to build the map using lidar and IMU. 

.. note:: To run the cartographer turtlebot3 example, source the following in the terminal.

::

    source /opt/ros/galactic/setup.bash
    source install/setup.bash
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=~/mbzirc_ws
    source ~/mbzirc_ws/install/setup.bash
    export TURTLEBOT3_MODEL=burger
    export GAZEBO_MODEL_PATH=~/mbzirc_ws/src/ros2_explorer/explorer_gazebo/models







