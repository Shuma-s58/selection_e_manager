from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('selection_e_manager'), 'config', 'params.yaml')
    params = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('use_sim_time')
    nodes = [Node(package='selection_e_manager', executable=e, output='screen',
                  parameters=[params, {'use_sim_time': sim}])
             for e in ('manager_node', 'camera_node', 'ocr_node', 'speaker_node')]
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *nodes,
    ])
