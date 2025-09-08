from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('f1tenth_teleop'),
                       'config', 'teleop_twist_joy.yaml')
    print(f"[teleop.launch] Using teleop YAML: {cfg}")
    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy', output='screen'),
        Node(package='teleop_twist_joy', executable='teleop_node', name='teleop',
             parameters=[cfg], output='screen'),
    ])
