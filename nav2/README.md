# mbzirc



ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast-port.sdf"


ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0  slot0:=mbzirc_planar_lidar slot1:=mbzirc_rgbd_camera 


ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0  slot0:=mbzirc_hd_camera
slot1:=mbzirc_planar_lidar

ros2 launch usv_description usv.launch.py

ros2 launch ros2_mapping map_bringsup.launch.py

python3 ~/mbzirc_ws/src/nav_packages/navigation/navigation/usv_odometry_publisher.py

ros2 run uav_llm_mission_planner uav_tf_publisher 

ros2 run uav_llm_mission_planner uav_mission_executor 

export PYTHONPATH=$PYTHONPATH:~/mbzirc_ws/src/nav_packages/usv_control/src

ros2 run usv_control twist_publisher 

python3 /home/muhayy/mbzirc_ws/src/nav_packages/navigation/navigation/navigator.py


ros2 launch unified_mission_planner heterogeneous_mission_system.launch.py 


-----------------------------------------------------
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 4 -r coast-port.sdf"

ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1450 y:=-16.5 z:=0.3 R:=0 P:=0 Y:=0  slot0:=mbzirc_planar_lidar slot1:=mbzirc_rgbd_camera 

ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_1 world:=coast model:=mbzirc_quadrotor x:=-1450 y:=-16.5 z:=4.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera


ros2 run uav_llm_mission_planner uav_tf_publisher 

ros2 run uav_llm_mission_planner uav_mission_executor 




python3 /home/muhayy/mbzirc_ws/src/llm_usv_navigation/llm_usv_navigation/usv_odometry_publisher.py 

 ros2 run usv_control twist_publisher 

python3 /home/muhayy/mbzirc_ws/src/nav_packages/navigation/navigation/navigator.py


ros2 launch unified_mission_planner heterogeneous_mission_system.launch.py 


ros2 launch mbzirc_ign spawn.launch.py name:=usv world:=coast model:=usv x:=-1385 y:=-72 z:=0.3 R:=0 P:=0 Y:=0  slot0:=mbzirc_planar_lidar slot1:=mbzirc_rgbd_camera 

ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_1 world:=coast model:=mbzirc_quadrotor  x:=-1385 y:=-72 z:=4.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera



ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_3 world:=coast model:=mbzirc_quadrotor x:=-1425 y:=-16.5 z:=10.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera

ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_5 world:=coast model:=mbzirc_quadrotor x:=-1355 y:=-16.5 z:=10.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera

ros2 launch mbzirc_ign spawn.launch.py name:=quadrotor_6 world:=coast model:=mbzirc_quadrotor x:=-1355 y:=0 z:=10.3 R:=0 P:=0 Y:=0 slot0:=mbzirc_hd_camera

