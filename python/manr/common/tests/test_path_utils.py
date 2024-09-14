import os

from manr.common.path_utils import create_temporary_directory, create_temporary_file
from manr.common.testing_utils import execute_pytest_file


def test_create_temporary_directory() -> None:
    with create_temporary_directory() as temp_dirpath:
        assert os.path.exists(temp_dirpath)

    # Directory should be deleted after the context manager exits.
    assert not os.path.exists(temp_dirpath)

    # Check arguments.
    prefix = "a"
    suffix = "b"

    with create_temporary_directory() as dirpath:
        with create_temporary_directory(
            prefix=prefix,
            suffix=suffix,
            dirpath=dirpath,
        ) as temp_dirpath:
            assert os.path.exists(temp_dirpath)
            assert temp_dirpath.startswith(dirpath)
            assert os.path.basename(temp_dirpath).startswith(prefix)
            assert os.path.basename(temp_dirpath).endswith(suffix)


def test_create_temporary_file():
    with create_temporary_file() as temp_file:
        assert os.path.exists(temp_file)

    # File should be deleted after the context manager exits.
    assert not os.path.exists(temp_file)


if __name__ == "__main__":
    execute_pytest_file()
