import os

import pytest

from python.lite6.utils.lite6_model_utils import (
    Lite6ModelType,
    get_drake_lite6_urdf_path,
)


def test_lite6_model_paths() -> None:
    """
    Test that each a valid and existing model path can be obtained
    for each lite6 model type.
    """
    for lite6_model_type in Lite6ModelType:
        urdf_path = get_drake_lite6_urdf_path(
            lite6_model_type=lite6_model_type,
        )
        assert os.path.isfile(urdf_path)


if __name__ == "__main__":
    pytest.main(["-s", "-v", __file__])
