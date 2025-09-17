#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    vesc_port_arg = DeclareLaunchArgument('vesc_port', default_value='/dev/ttyACM0')
    
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.01,
            'autorepeat_rate': 20.0,
        }]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('mushr_configs'),
                'config', 'joy_teleop_fixed.yaml'
            ])
        ]
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('mushr_configs'),
                'config', 'vesc_4_12_mini.yaml'
            ]),
            {'port': LaunchConfiguration('vesc_port')}
        ]
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('mushr_configs'),
                'config', 'ackermann_to_vesc_7_2v.yaml'
            ])
        ]
    )

    return LaunchDescription([
        joy_dev_arg,
        vesc_port_arg,
        joy_node,
        joy_teleop_node,
        vesc_driver_node,
        ackermann_to_vesc_node,
    ])
