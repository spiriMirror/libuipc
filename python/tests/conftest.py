# Shared pytest configuration and helpers for python/tests.
import sys

def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "basic: mark test as a basic/smoke test (e.g. run with -m basic)",
    )
    config.addinivalue_line(
        "markers",
        "example: mark test as an example / demo (may open GUI)",
    )


# Skip tests that use uipc.Engine("cuda", ...) on macOS (CUDA is not available).
skip_cuda_on_macos = sys.platform == "darwin"
skip_cuda_on_macos_reason = "CUDA not available on macOS"
