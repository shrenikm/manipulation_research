import os

from manr.common.path_utils import create_temporary_directory, create_temporary_file
from manr.common.testing_utils import execute_pytest_file


def test_create_temporary_directory() -> None:
    with create_temporary_directory() as temp_dirpath:
        assert os.path.isdir(temp_dirpath)

    # Directory should be deleted after the context manager exits.
    assert not os.path.isdir(temp_dirpath)

    # Check arguments.
    prefix = "a"
    suffix = "b"

    with create_temporary_directory() as dirpath:
        with create_temporary_directory(
            prefix=prefix,
            suffix=suffix,
            dirpath=dirpath,
        ) as temp_dirpath:
            assert os.path.isdir(temp_dirpath)
            assert temp_dirpath.startswith(dirpath)
            assert os.path.basename(temp_dirpath).startswith(prefix)
            assert os.path.basename(temp_dirpath).endswith(suffix)

        assert not os.path.isdir(temp_dirpath)
    assert not os.path.isdir(dirpath)


def test_create_temporary_file():
    with create_temporary_file() as temp_filepath:
        assert os.path.exists(temp_filepath)

    # File should be deleted after the context manager exits.
    assert not os.path.exists(temp_filepath)

    prefix = "a"
    suffix = ".b"

    with create_temporary_directory() as dirpath:
        with create_temporary_file(
            prefix=prefix,
            suffix=suffix,
            dirpath=dirpath,
        ) as temp_filepath:
            assert os.path.isfile(temp_filepath)
            assert temp_filepath.startswith(dirpath)
            assert os.path.basename(temp_filepath).startswith(prefix)
            assert os.path.basename(temp_filepath).endswith(suffix)


if __name__ == "__main__":
    execute_pytest_file()
