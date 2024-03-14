import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pcd2pgm'), 'config', 'pcd.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pcd2pgm_node = Node(
        package='pcd2pgm',
        executable='pcd2pgm_node',
        output='screen',
        parameters=[
            config, 
            {'use_sim_time': use_sim_time }
        ]
    )

    return LaunchDescription([pcd2pgm_node])
