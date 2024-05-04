from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import launch_ros.descriptions

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='assesment',
            namespace='assesment',
            executable='path',
            name='path'
        ),
        Node(
            package='assesment',
            namespace='assesment',
            executable='map',
            name='map',
            parameters = [get_package_share_directory('assesment') + '/config/map.yaml'])
    ])
