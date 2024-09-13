"""
Utilities for working with paths.
"""
import os
from typing import List

from manr.common.custom_types import DirPath, FilePath


def list_directories_in_path(
    path: DirPath,
) -> List[DirPath]:
    return [
        file_or_directory
        for file_or_directory in os.listdir(path)
        if os.path.isdir(os.path.join(path, file_or_directory))
    ]


def list_files_in_path(
    path: DirPath,
) -> List[FilePath]:
    return [
        file_or_directory
        for file_or_directory in os.listdir(path)
        if os.path.isfile(os.path.join(path, file_or_directory))
    ]
