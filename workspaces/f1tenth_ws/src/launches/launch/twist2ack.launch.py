from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Subscribes: /cmd_vel (Twist)
        # Publishes:  /cmd_teleop (Ackermann) → consumed by ackermann_mux per your YAML
        Node(
            package='f1tenth_teleop',
            executable='twist_to_ackermann',
            name='twist_to_ackermann',
            output='screen',
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('ackermann_cmd', '/cmd_teleop'),
            ],
            # If you later add a YAML for this node, you can pass it here.
            parameters=[]
        )
    ])
