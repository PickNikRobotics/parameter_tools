import yaml
from pathlib import Path
from typing import List, Union
import xacro


class ParameterBuilderFileNotFoundError(KeyError):
    pass


# Parameter value types
ParameterValueType = Union[
    str,
    int,
    float,
    bool,
    List[str],
    List[int],
    List[float],
    List[bool],
    bytes,
]


def raise_if_file_not_found(file_path: Path):
    if not file_path.exists():
        raise ParameterBuilderFileNotFoundError(f"File {file_path} doesn't exists")


def load_file(file_path: Path):
    raise_if_file_not_found(file_path)
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(file_path: Path):
    raise_if_file_not_found(file_path)

    try:
        with open(file_path, "r") as file:
            return yaml.load(file, Loader=yaml.FullLoader)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_xacro(file_path: Path, mappings: dict = None):
    raise_if_file_not_found(file_path)

    file = xacro.process_file(file_path, mappings=mappings)
    return file.toxml()
