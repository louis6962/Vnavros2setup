import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

WS = os.path.expanduser('~/f1tenth_ws/src')

MUX_TOPICS = os.path.join(WS, 'ackermann_mux', 'config', 'ackermann_mux_topics.yaml')
MUX_LOCKS  = os.path.join(WS, 'ackermann_mux', 'config', 'ackermann_mux_locks.yaml')
ACK2VESC   = os.path.join(WS, 'vesc', 'vesc_ackermann', 'config', 'ack2vesc.yaml')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        mux_launch,
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        
        Node(
            package='vesc_ackermann', executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node', output='screen',
            remappings=[('cmd','ackermann_cmd')],        # <- mux output
            parameters=[ACK2VESC, {'use_sim_time': use_sim_time}]
        ),
    ])
