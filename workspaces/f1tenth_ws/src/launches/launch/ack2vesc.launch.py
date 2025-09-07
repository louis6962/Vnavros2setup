from launch import LaunchDescription
from launch_ros.actions import Node
import os

def _first_existing(paths):
    for p in paths:
        if p and os.path.exists(p):
            return p
    return None

def generate_launch_description():
    # Prefer your source YAML (keeps your tuned values)
    src = os.path.expanduser(
        '~/Vnavros2setup-pr2/workspaces/f1tenth_ws/src/vesc/vesc_ackermann/config/ack2vesc.yaml'
    )
    # Fallbacks if needed
    alt = os.path.expanduser('~/f1tenth_ws/src/vesc/vesc_ackermann/config/ack2vesc.yaml')
    try:
        from ament_index_python.packages import get_package_share_directory
        inst = os.path.join(get_package_share_directory('vesc_ackermann'), 'config', 'ack2vesc.yaml')
    except Exception:
        inst = None

    cfg = _first_existing([src, alt, inst])
    params = [cfg] if cfg else []

    return LaunchDescription([
        Node(
            package='vesc_ackermann', executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node', output='screen',
            parameters=params,
            # Subscribe to the fixed topic your chain publishes:
            remappings=[('cmd', '/ackermann_cmd')]
        )
    ])
