from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    vesc_port = LaunchConfiguration('vesc_port')
    joy_dev   = LaunchConfiguration('joy_dev')
    ack_cmd   = LaunchConfiguration('ackermann_cmd')
    twist_cmd = LaunchConfiguration('twist_cmd')

    return LaunchDescription([
        DeclareLaunchArgument('vesc_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('joy_dev',   default_value='/dev/input/js0'),
        DeclareLaunchArgument('ackermann_cmd', default_value='/cmd_ackermann'),
        DeclareLaunchArgument('twist_cmd',     default_value='/cmd_vel'),

        # 1) Joystick driver
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{'dev': joy_dev}]
        ),

        # 2) Joystick → Twist (always enabled)
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy',
            remappings=[('cmd_vel', twist_cmd)],
            parameters=[{'enable_button': -1}]
        ),

        # 3) Twist → Ackermann (check exec name below; this is common)
        Node(
            package='vesc_ackermann', executable='twist_to_ackermann',
            name='twist_to_ackermann',
            remappings=[('twist', twist_cmd), ('cmd', ack_cmd)]
        ),

        # 4) Ackermann → VESC
        Node(
            package='vesc', executable='ackermann_to_vesc_node', name='ackermann_to_vesc',
            remappings=[('input', ack_cmd)]
        ),

        # 5) VESC driver
        Node(
            package='vesc', executable='vesc_driver_node', name='vesc_driver',
            parameters=[{'port': vesc_port, 'baud': 115200}]
        ),
    ])
