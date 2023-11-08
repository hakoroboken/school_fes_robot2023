import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
     package_dir = get_package_share_directory('haya_imu_ros2')
     return launch.LaunchDescription([
          Node(package='haya_imu_ros2', executable='haya_imu_node', name='haya_imu_node', parameters=[os.path.join(package_dir, 'config', 'params.yaml')], output='screen'),
          Node(package='haya_imu_ros2', executable='haya_topic_echo', name='haya_topic_echo', output='screen')
     ])
