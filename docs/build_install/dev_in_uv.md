# Development in UV

[Install UV](https://docs.astral.sh/uv/getting-started/installation/) if you haven't already.

Install vcpkg as [Windows](./windows.md) or [Linux](./linux.md) guides.

Install the build dependencies:

```bash
uv pip install scikit-build-core setuptools-scm
```

Build the project and install the Python package in editable mode; specify the build directory to cache the CMake build results; and disable build isolation to prevent always reinstalling the dependencies:

```bash
uv pip install -e . --config-settings=build-dir=build --no-build-isolation -v
```

Run the tests, with `--no-sync` to prevent uv from rebuilding the project every time you run the tests (which is slow):

```bash
uv run --no-sync pytest python/tests
```
