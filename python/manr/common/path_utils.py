"""
Utilities for working with paths.
"""

import os
import shutil
import tempfile
from contextlib import contextmanager
from typing import Generator, List, Optional

from manr.common.custom_types import DirPath, FilePath


@contextmanager
def create_temporary_directory(
    prefix: Optional[str] = None,
    suffix: Optional[str] = None,
    dirpath: Optional[DirPath] = None,
) -> Generator[DirPath, None, None]:
    """
    Creates a temporary directory, yields the path to the directory, and then deletes the directory.
    """

    temp_dir = tempfile.mkdtemp(
        prefix=prefix,
        suffix=suffix,
        dir=dirpath,
    )
    try:
        yield temp_dir
    finally:
        shutil.rmtree(temp_dir)


def list_directories_in_path(
    path: DirPath,
) -> List[DirPath]:
    """
    Returns a list of all directories in the given path.
    """
    return [
        file_or_directory
        for file_or_directory in os.listdir(path)
        if os.path.isdir(os.path.join(path, file_or_directory))
    ]


def list_files_in_path(
    path: DirPath,
) -> List[FilePath]:
    """
    Returns a list of all files in the given path.
    """
    return [
        file_or_directory
        for file_or_directory in os.listdir(path)
        if os.path.isfile(os.path.join(path, file_or_directory))
    ]


def create_directory_if_not_exists(
    directory_path: DirPath,
) -> None:
    """
    Creates a directory at the given path if it does not already exist.
    """
    os.makedirs(directory_path, exist_ok=True)
