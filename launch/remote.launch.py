import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo

def generate_launch_description():

    remote_config = os.path.join(get_package_share_directory('remote'), 'params', 'remote_cfg.yaml')

    sbus_node = Node(
        package='remote',
        executable='remote_node',
        name='remote_node',
        output='screen',
        parameters=[remote_config],
        namespace='/',
        )
    
    return LaunchDescription([
        sbus_node,
    ])
