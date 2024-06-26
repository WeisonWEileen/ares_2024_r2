import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("rc_vision_bringup"), "launch")
)

node_params = os.path.join(
    get_package_share_directory("rc_vision_bringup"), "config", "node_params.yaml"
)


def generate_launch_description():

    # from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin,name):
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
            ros_arguments=[
                "--ros-args",
                # "--log-level",
            ],
            on_exit=Shutdown(),
        )

    v4l2_camera_node = get_camera_node(
        "v4l2_camera", "v4l2_camera::V4L2Camera", "v4l2_camera_node"
    )
    detecor_node = get_camera_node("armor_detector", "rc_auto_aim::InferencerNode","detector_node")
    projector_node = get_camera_node("armor_detector", "rc_auto_aim::ProjectorNode","projector_node")
    cam_detector = get_camera_detector_projector_container(
        # v4l2_camera_node, 
        detecor_node, 
        # projector_node
    )

    # mv_camera_node = get_camera_node(
    #     "mindvision_camera", "mindvision_camera::MVCameraNode"
    # )

    # @TODO: Add support for multiple cameras: including realsense D435
    # if launch_params["camera"] == "hik":
    # elif launch_params["camera"] == "mv":
    # cam_detector = get_camera_detector_container(mv_camera_node)

    # serial_driver_node = Node(
    #     package="rm_serial_driver",
    #     executable="rm_serial_driver_node",
    #     name="serial_driver",
    #     output="both",
    #     emulate_tty=True,
    #     parameters=[node_params],
    #     on_exit=Shutdown(),
    #     ros_arguments=[
    #         "--ros-args",
    #         "--log-level",
    #         "serial_driver:=" + launch_params["serial_log_level"],
    #     ],
    # )

    # delay_serial_node = TimerAction(
    #     period=1.5,
    #     actions=[serial_driver_node],
    # )

    # delay_tracker_node = TimerAction(
    #     period=2.0,
    #     actions=[tracker_node],
    # )
    # realsense并没有component的选项
    
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
    return LaunchDescription(
        [
            cam_detector, 
            realsense_launch
        ])
