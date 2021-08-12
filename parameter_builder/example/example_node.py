#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.parameter import ParameterType
from rcl_interfaces.msg import ParameterDescriptor


class ParameterBuilderExample(Node):
    def __init__(self):
        super().__init__("parameter_builder_example")

        self.declare_parameter(
            "my_parameter",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )
        self.declare_parameter(
            "parameter_file",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "ros2_versions",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            "the_answer_to_life",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER),
        )
        self.declare_parameter(
            "package_name",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            "my_robot",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

        self.get_logger().info(
            f"my_parameter: {self.get_parameter('my_parameter').value}"
        )
        self.get_logger().info(
            f"parameter_file: {self.get_parameter('parameter_file').value}"
        )
        self.get_logger().info(f"my_robot: {self.get_parameter('my_robot').value}")
        self.get_logger().info(
            f"ros2_versions: {self.get_parameter('ros2_versions').value}"
        )
        self.get_logger().info(
            f"the_answer_to_life: {self.get_parameter('the_answer_to_life').value}"
        )
        self.get_logger().info(
            f"package_name: {self.get_parameter('package_name').value}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = ParameterBuilderExample()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
