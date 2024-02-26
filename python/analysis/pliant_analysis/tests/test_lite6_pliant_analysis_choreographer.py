import os

from python.analysis.pliant_analysis.lite6_pliant_analysis_choreographer import (
    Lite6PliantChoreographer,
)
from python.common.testing_utils import execute_pytest_file


def test_choreographer_from_yaml() -> None:

    yaml_filepath = os.path.join(
        os.path.dirname(__file__), "..", "choreographer_config.yaml"
    )
    choreographer = Lite6PliantChoreographer.from_yaml(
        yaml_filepath=yaml_filepath,
    )
    print(choreographer)


if __name__ == "__main__":
    execute_pytest_file()
