import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # config = os.path.join(
    #     get_package_share_directory('armor_detector'), 'config', 'serial_driver.yaml')

    rc_armor_detector_node = Node(
        package='armor_detector',
        executable='rc_armor_detector_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        # parameters=[config],
    )

    return LaunchDescription([rc_armor_detector_node])
