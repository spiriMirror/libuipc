# PYUIPC

the python binding for libuipc

## Build

Build the wheel:

```bash 
uv sync --no-dev
uv build
```

## Dev

```bash 
uv sync
```

### CMake presets (recommended)

Build the C++ bindings for local development:

```bash
cmake --preset pybind-dev
cmake --build --preset pybind-dev
```

Set the Python interpreter explicitly if needed:

```bash
PYTHON=/path/to/python cmake --preset pybind-dev
cmake --build --preset pybind-dev
```

### Editable install

Install the Python package in editable mode (will build native code via CMake):

```bash
pip install -e ./python
```