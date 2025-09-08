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
    installed = os.path.join(get_package_share_directory('vesc_driver'),
                             'params', 'vesc_config.yaml')
    ws = os.environ.get('WS', os.path.expanduser('~/Vnavros2setup/workspaces/f1tenth_ws'))
    src = os.path.join(ws, 'src', 'vesc', 'vesc_driver', 'params', 'vesc_config.yaml')

    cfg = first_existing([src, installed])  # prefer your source YAML (has duty_cycle_max: 0.20)
    if not cfg:
        raise RuntimeError(f"vesc_config.yaml not found in:\n- {src}\n- {installed}")
    print(f"[driver.launch] Using YAML: {cfg}")

    return LaunchDescription([
        Node(
            package='vesc_driver', executable='vesc_driver_node', name='vesc_driver',
            parameters=[cfg],  # YAML sets port:/dev/ttyACM0 and duty_cycle_max:0.20
            output='screen'
        )
    ])
