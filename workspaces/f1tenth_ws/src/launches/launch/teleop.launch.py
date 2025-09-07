from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Use your PR2 YAML (has enable_button: 9, top key: teleop)
    cfg = os.path.expanduser('~/Vnavros2setup-pr2/workspaces/f1tenth_ws/src/f1tenth_teleop/config/teleop_twist_joy.yaml')
    if not os.path.exists(cfg):
        raise RuntimeError(f"[teleop.launch] YAML not found: {cfg}")

    return LaunchDescription([
        Node(
            package='joy', executable='joy_node', name='joy',
            parameters=[{'device': '/dev/input/js0'}],
            output='screen'
        ),
        # Node name must match YAML top key: teleop
        Node(
            package='teleop_twist_joy', executable='teleop_node', name='teleop',
            parameters=[cfg],
            output='screen'
        ),
    ])
