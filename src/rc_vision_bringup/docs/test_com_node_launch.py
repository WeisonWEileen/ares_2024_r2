import os
import yaml
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    # Get the filepath to your config file
    configFilepath = os.path.join(
        get_package_share_directory("rc_vision_bringup"), "config", "node_params.yaml"
    )

    # Load the parameters specific to your ComposableNode
    with open(configFilepath, "r") as file:
        configParams = yaml.safe_load(file)["detector_node"]["ros__parameters"]

    # Create your ComposableNode
    container = ComposableNodeContainer(
        name="cam_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",  # This name must match your CMakelist file
        composable_node_descriptions=[
            ComposableNode(
                package="rc_detector",
                plugin="rc_detector::InferencerNode",
                name="detector_node",
                parameters=[configParams],
            )
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])
