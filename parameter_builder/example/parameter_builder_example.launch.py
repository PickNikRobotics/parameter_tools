from parameter_builder import ParameterBuilder

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="parameter_builder",
                executable="example_node.py",
                parameters=[
                    ParameterBuilder("parameter_builder")
                    .parameter("my_parameter", 20.0)
                    .file_parameter(
                        "parameter_file", "config/parameter_file"
                    )  # Or /absolute/path/to/file
                    .yaml(file_path="config/parameters.yaml")  # Or /absolute/path/to/file
                    .xacro_parameter(
                        parameter_name="my_robot",
                        file_path="config/parameter.xacro",  # Or /absolute/path/to/file
                        mappings={"prefix": "robot"},
                    )
                    .to_dict()
                ],
            )
        ]
    )
