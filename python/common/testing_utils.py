from typing import Optional
import pytest

def execute_pytest(
    file: str,
    test_name: Optional[str] = None,
) -> None:
    pytest_flags = ["-s", "-v"]
    if test_name is None:
        pytest.main(pytest_flags +  [file])
    else:
        pytest.main(pytest_flags + ["-k", test_name])

