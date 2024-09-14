import os

from manr.common.path_utils import create_temporary_directory
from manr.common.testing_utils import execute_pytest_file


def test_create_temporary_directory():
    with create_temporary_directory("prefix") as temp_dir:
        assert os.path.exists(temp_dir)

    # Directory should be deleted after the context manager exits.
    assert not os.path.exists(temp_dir)


if __name__ == "__main__":
    execute_pytest_file()
