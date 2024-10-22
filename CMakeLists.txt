cmake_minimum_required(VERSION 3.26)
# set cmake include path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(cmake/uipc_utils.cmake)

# =========================================================================
# Project Options
# =========================================================================
option(UIPC_USING_LOCAL_VCPKG "Using local vcpkg" ON)
option(UIPC_BUILD_GUI "Build GUI, turn it off if you're server only" ON)
option(UIPC_BUILD_PYBIND "Build pyuipc" OFF)
option(UIPC_BUILD_TORCH_EXTENSION "Build pytorch extension" OFF)
option(UIPC_BUILD_EXAMPLES "Build examples" ON)
option(UIPC_BUILD_TESTS "Build tests" ON)
option(UIPC_BUILD_BENCHMARKS "Build benchmarks" ON)

# =========================================================================
# Backend Options
# =========================================================================
option(UIPC_WITH_CUDA_BACKEND "Build with CUDA backend" ON)


uipc_show_logo("0.0.1")
uipc_show_options()
uipc_config_vcpkg_install()
project(uipc)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# dependencies
add_subdirectory(external)

# uipc
add_subdirectory(src)

# apps
add_subdirectory(apps)