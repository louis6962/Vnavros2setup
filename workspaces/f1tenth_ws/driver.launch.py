from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('vesc_driver'),'params','vesc_config.yaml')
    return LaunchDescription([
        Node(package='vesc_driver', executable='vesc_driver_node', name='vesc_driver',
             parameters=[cfg, {'port':'/dev/ttyACM0'}], output='screen')
    ])
