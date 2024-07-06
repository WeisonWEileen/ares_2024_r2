import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("rc_vision_bringup"), "launch")
)

node_params = os.path.join(
    get_package_share_directory("rc_vision_bringup"), "config", "realsense.yaml"
)



def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_composable_node(package, plugin, name):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    def get_camera_detector_projector_container(*regestered_nodes):
        return ComposableNodeContainer(
            name="cam_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=list(regestered_nodes),
            output="both",
            emulate_tty=True,
            parameters=[node_params],
            ros_arguments=[
                "--ros-args",
                # "--log-level",
            ],
            on_exit=Shutdown(),
    )

    # --------------------------------------#
    # --------composable_node part----------#
    v4l2_camera_node = get_composable_node(
        "v4l2_camera", "v4l2_camera::V4L2Camera", "v4l2_camera_node"
    )

    inferencer_node = get_composable_node(
        "rc_detector", "rc_detector::InferencerNode", "inferencer_node"
    )

    projector_node = get_composable_node(
        "rc_detector", "rc_detector::ProjectorNode", "projector_node"
    )

    # 总的接口

    cam_detector = get_camera_detector_projector_container(

        v4l2_camera_node,
        # inferencer_node,
        # projector_node
    )
    # --------composable_node part----------#
    # --------------------------------------#

    # @TODO: Add support for multiple cameras: including realsense D435
    # if launch_params["camera"] == "hik":
    # elif launch_params["camera"] == "mv":
    # cam_detector = get_camera_detector_container(mv_camera_node)

    # -----------------------------#
    # --------delay part-----------#

    # delay_serial_node = TimerAction(
    #     period=1.5,
    #     actions=[serial_driver_node],
    # )

    # delay_tracker_node = TimerAction(
    #     period=2.0,
    #     actions=[tracker_node],
    # )

    # --------delay part-----------#
    # -----------------------------#

    # -----------------------------#
    # --------realsense part-------#
    # realsense好像并没有component的选项？
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
                )
            ]
        ),
        launch_arguments={"align_depth.enable": "true"}.items(),
    )
    # --------realsense part-------#
    # -----------------------------#

    # ------------------------------#
    # --------serial driver---------#
    # rc_serial_driver_node = Node(
    #     package='rc_serial_driver',
    #     executable='rc_serial_driver_node',
    #     namespace='',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[node_params],
    # )
    # --------serial driver---------#
    # ------------------------------#

    return LaunchDescription(
        [
            # realsense_launch,
            cam_detector
        ]
    )
