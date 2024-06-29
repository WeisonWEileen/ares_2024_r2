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

    rc_detector_node = Node(
        package="armor_detector",
        executable="rc_armor_detector_node",
        name="rc_armor_detector_node",
        output="screen",
        emulate_tty=True,
        parameters=[config],
        ros_arguments=["--ros-args"],
    )

    rc_projector_node = Node(
        package="armor_detector",
        executable="rc_armor_projector_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        # parameters=[config],
    )

    #第二版r2的底部摄像头 
    carry_state_node = Node(
        package="armor_detector",
        executable="rc_carry_state_node",
        namespace="",
        output="screen",
        emulate_tty=True,
        parameters=[{
            'camera_port': "/dev/video0" ,# Default camera port, can be overridden by launch arguments
            'roi':[190,250,480,550], #row range(a,b),col range(c,d)The ROI to detect whether the robot is carrying the ball 
            'thres': 3800,#The threshold of the sum of the pixel values in the ROI
            'fps':20,

            }],
        arguments=['--ros-args', '--log-level', 'debug'],
    )

    ld = LaunchDescription()
    # ld.add_action(realsense_launch)
    ld.add_action(rc_projector_node)
    # ld.add_action(carry_state_node)

    return ld
