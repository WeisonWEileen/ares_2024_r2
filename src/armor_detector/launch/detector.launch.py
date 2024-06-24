import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("armor_detector"), "config", "detector.yaml"
    )

    # realsense_launch = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([
    #     PathJoinSubstitution([
    #         FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
    # launch_arguments={
    #     'align_depth.enable': 'true'
    #     }.items(),
    # )

    # rc_detector_node = Node(
    #     package="armor_detector",
    #     executable="rc_armor_detector_node",
    #     name="rc_armor_detector_node",
    #     output="screen",
    #     emulate_tty=True,
    #     parameters=[config],
    #     ros_arguments=["--ros-args"],
    # )

    rc_projector_node = Node(
        package="armor_detector",
        executable="rc_armor_projector_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        # parameters=[config],
    )

    ld = LaunchDescription()
    # ld.add_action(realsense_launch)
    ld.add_action(rc_projector_node)
    # ld.add_action(rc_projector_node)

    return ld
