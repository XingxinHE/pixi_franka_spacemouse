import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def load_yaml(file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"File not found: {file_path}")
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def generate_nodes(context):
    config_file_name = LaunchConfiguration("config_file").perform(context)
    package_config_dir = FindPackageShare("spacemouse_publisher").perform(context)
    config_file = os.path.join(package_config_dir, "config", config_file_name)
    configs = load_yaml(config_file)
    nodes = []
    for item_name, config in configs.items():
        node_parameters = {k: v for k, v in config.items() if k != "namespace"}
        nodes.append(
            Node(
                package="spacemouse_publisher",
                executable="pyspacemouse_publisher",
                name="spacemouse_publisher",
                namespace=str(config["namespace"]),
                output="screen",
                parameters=[node_parameters],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="example_fr3_config.yaml",
                description="Name of the spacemouse configuration file to load",
            ),
            OpaqueFunction(function=generate_nodes),
        ]
    )
