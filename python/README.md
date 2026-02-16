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

#### Specifying a Custom Build Directory

To specify a custom build directory for caching C++ builds and faster rebuilds:

```bash
pip install -e . --config-settings=build-dir=build
```

Or reuse an existing CMake build directory:

```bash
pip install -e . --config-settings=build-dir=build/Release
```

For in-place builds (fastest for development):

```bash
pip install -e . --config-settings=editable.mode=inplace
```

The build directory configuration is particularly useful for:
- Reusing existing CMake builds
- Avoiding full rebuilds after `pip install -e`
- Speeding up CUDA compilation during development