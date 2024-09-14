"""
Utilities for working with paths.
"""

import os
from typing import List

from manr.common.custom_types import DirPath, FilePath


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
