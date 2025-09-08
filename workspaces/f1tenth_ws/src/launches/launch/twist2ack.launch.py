from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='f1tenth_teleop', executable='twist_to_ackermann',
             name='twist_to_ackermann',
             remappings=[('cmd_vel', 'cmd_vel'),
                         ('ackermann_cmd', 'ackermann_cmd')],
             parameters=[{'wheelbase': 0.33}],
             output='screen')
    ])
