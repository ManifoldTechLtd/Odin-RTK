from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('odin_rtk_fusion')

    default_config = os.path.join(pkg_share, 'config', 'fusion_params.yaml')
    default_rviz = os.path.join(pkg_share, 'ros', 'rviz', 'fusion_ros2.rviz')

    config_file = LaunchConfiguration('config_file')
    output_project_topic = LaunchConfiguration('output_project_topic')
    output_project_frame_id = LaunchConfiguration('output_project_frame_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
        ),
        DeclareLaunchArgument(
            'output_project_topic',
            default_value='/world/odom_projected',
        ),
        DeclareLaunchArgument(
            'output_project_frame_id',
            default_value='world_proj',
        ),
        Node(
            package='odin_rtk_fusion',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                'config_file': config_file,
                'output_project_topic': output_project_topic,
                'output_project_frame_id': output_project_frame_id,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizvisualisation',
            output='log',
            arguments=['-d', default_rviz],
        ),
    ])
