from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='point_cloud_filters',
            executable='cloud_filter_chain_node',
            parameters=[
                PathJoinSubstitution([FindPackageShare('three_d_laser_pc_filter'), 'config', 'example.yaml',])
            ],
        )
    ])
