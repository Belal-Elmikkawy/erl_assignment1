import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'erl1_assignment1'


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory(PKG), 'config', 'assignment_sim.yaml')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='assignment_sim.yaml (Gazebo) or assignment_rosbot.yaml (real robot)')

    detector = Node(package=PKG, executable='aruco_detector_node.py', name='aruco_detector_node',
                    output='screen', parameters=[LaunchConfiguration('params_file')])
    hunter = Node(package=PKG, executable='treasure_hunter_node.py', name='treasure_hunter_node',
                  output='screen', parameters=[LaunchConfiguration('params_file')])
    return LaunchDescription([params_arg, detector, hunter])