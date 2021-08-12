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
                    .file_parameter("parameter_file", "config/parameter_file")
                    .yaml("config/parameters.yaml")
                    .xacro_parameter(
                        parameter_name="my_robot",
                        file_path="config/parameter.xacro",
                        mappings={"prefix": "robot"},
                    )
                    .to_dict()
                ],
            )
        ]
    )
