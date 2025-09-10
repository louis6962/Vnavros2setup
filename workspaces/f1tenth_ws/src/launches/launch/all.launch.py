import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WS = os.path.expanduser('~/f1tenth_ws/src')

TELEOP_YAML = os.path.join(WS, 'f1tenth_teleop', 'config', 'teleop_twist_joy.yaml')
MUX_TOPICS  = os.path.join(WS, 'ackermann_mux', 'config', 'ackermann_mux_topics.yaml')
MUX_LOCKS   = os.path.join(WS, 'ackermann_mux', 'config', 'ackermann_mux_locks.yaml')
ACK2VESC    = os.path.join(WS, 'vesc', 'vesc_ackermann', 'config', 'ack2vesc.yaml')
VESC_YAML   = os.path.join(WS, 'vesc', 'vesc_driver', 'config', 'vesc_driver.yaml')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_dev    = LaunchConfiguration('joy_dev')
    vesc_port  = LaunchConfiguration('vesc_port')
    vesc_baud  = LaunchConfiguration('vesc_baud')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy_dev',      default_value='/dev/input/js0'),
        DeclareLaunchArgument('vesc_port',    default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('vesc_baud',    default_value='115200'),
        DeclareLaunchArgument('base_frame',   default_value='base_link'),

        Node(package='joy', executable='joy_node', name='joy', output='screen',
             parameters=[{'device': joy_dev, 'deadzone': 0.05,
                          'autorepeat_rate': 20.0, 'use_sim_time': use_sim_time}]),

        Node(package='teleop_twist_joy', executable='teleop_node',
             name='teleop', output='screen',
             parameters=[TELEOP_YAML, {'use_sim_time': use_sim_time}]),

        Node(package='f1tenth_teleop', executable='twist_to_ackermann',
             name='twist_to_ackermann', output='screen',
             remappings=[('cmd_vel','cmd_vel'), ('cmd','cmd_vel'),
                         ('ackermann_cmd','cmd_teleop')],
             parameters=[{'use_sim_time': use_sim_time}]),

        Node(package='ackermann_mux', executable='ackermann_mux',
             name='ackermann_mux', output='screen',
             parameters=[MUX_TOPICS, MUX_LOCKS, {'use_sim_time': use_sim_time}]),

        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node',
             name='ackermann_to_vesc_node', output='screen',
             remappings=[('cmd','ackermann_cmd')],
             parameters=[ACK2VESC, {'use_sim_time': use_sim_time}]),

        Node(package='vesc_driver', executable='vesc_driver_node',
             name='vesc_driver', output='screen',
             parameters=[VESC_YAML,
                         {'port': vesc_port, 'baud': vesc_baud, 'interface':'serial',
                          'frame_id': base_frame, 'use_sim_time': use_sim_time}]),
    ])
