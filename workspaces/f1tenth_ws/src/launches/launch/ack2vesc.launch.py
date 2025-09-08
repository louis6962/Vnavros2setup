from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def first_existing(paths):
    for p in paths:
        if p and os.path.isfile(p):
            return p
    return None

def generate_launch_description():
    installed = os.path.join(get_package_share_directory('vesc_ackermann'),
                             'config', 'ack2vesc.yaml')
    ws = os.environ.get('WS', os.path.expanduser('~/Vnavros2setup/workspaces/f1tenth_ws'))
    src = os.path.join(ws, 'src', 'vesc', 'vesc_ackermann', 'config', 'ack2vesc.yaml')

    cfg = first_existing([installed, src])
    if not cfg:
        raise RuntimeError(f"ack2vesc.yaml not found in:\n- {installed}\n- {src}")
    print(f"[ack2vesc.launch] Using YAML: {cfg}")

    return LaunchDescription([
        Node(
            package='vesc_ackermann', executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            parameters=[cfg],
            remappings=[('cmd', 'ackermann_cmd')],
            output='screen'
        )
    ])
