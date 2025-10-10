from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='quadrotor_1',
        description='Namespace for the UAV'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level for the nodes'
    )
    
    # UAV Navigation Node
    uav_navigation_node = Node(
        package='uav_navigation',
        executable='uav_navigation_node',
        name='uav_navigation_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True
    )
    
    # UAV Odometry TF Broadcaster Node
    uav_tf_broadcaster_node = Node(
        package='uav_navigation',
        executable='uav_odom_tf_broadcaster',
        name='uav_tf_broadcaster',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True
    )
    
    return LaunchDescription([
        uav_tf_broadcaster_node,
        #namespace_arg,
        #use_sim_time_arg,
        #log_level_arg
        uav_navigation_node
    ])