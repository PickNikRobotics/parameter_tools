from pathlib import Path

from .utils import load_file, load_yaml, load_xacro, ParameterValueType

from ament_index_python.packages import get_package_share_directory


class ParameterBuilder(object):
    _package_path = None
    _parameters = {}

    def __init__(self, package_name: str):
        self._package_path = Path(get_package_share_directory(package_name))

    def yaml(self, file_path: str, parameter_namespace: str = None):
        if parameter_namespace:
            if parameter_namespace in self._parameters:
                self._parameters[parameter_namespace].update(load_yaml(self._package_path / file_path))
            else:
                self._parameters[parameter_namespace] = load_yaml(self._package_path / file_path)
        else:
            self._parameters.update(load_yaml(self._package_path / file_path))
        return self

    def file_parameter(self, parameter_name: str, file_path: str):
        self._parameters[parameter_name] = load_file(self._package_path / file_path)
        return self

    def xacro_parameter(
        self, parameter_name: str, file_path: str, mappings: dict = None
    ):
        self._parameters[parameter_name] = load_xacro(
            self._package_path / file_path, mappings=mappings
        )
        return self

    def parameter(self, parameter_name: str, parameter_value: ParameterValueType):
        self._parameters[parameter_name] = parameter_value
        return self

    def to_dict(self):
        return self._parameters
