cmake_minimum_required(VERSION 3.26)
# set cmake include path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(cmake/uipc_utils.cmake)

# =========================================================================
# Project Version
# =========================================================================
set(UIPC_VERSION_MAJOR 0)
set(UIPC_VERSION_MINOR 9)
set(UIPC_VERSION_PATCH 0)

# =========================================================================
# Project Options
# =========================================================================
option(UIPC_USING_LOCAL_VCPKG "Using local vcpkg" ON)
option(UIPC_BUILD_GUI "Build GUI, turn it off if you're server only" OFF)
option(UIPC_BUILD_PYBIND "Build pyuipc" OFF)
option(UIPC_BUILD_TORCH_EXTENSION "Build pytorch extension" OFF)
option(UIPC_BUILD_EXAMPLES "Build examples" ON)
option(UIPC_BUILD_TESTS "Build tests" ON)
option(UIPC_BUILD_BENCHMARKS "Build benchmarks" ON)
option(UIPC_DEV_MODE "Enable developer mode" OFF)
set(UIPC_PYTHON_EXECUTABLE_PATH "" CACHE PATH "Python executable path")

# =========================================================================
# Backend Options
# =========================================================================
option(UIPC_WITH_CUDA_BACKEND "Build with CUDA backend" ON)

# =========================================================================
# Show Logo and Options
# =========================================================================
uipc_show_logo()
uipc_find_python_executable_path()
uipc_show_options()
uipc_config_vcpkg_install()


# =========================================================================
#
# Libuipc Project
#
# =========================================================================

project(uipc)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json

# dependencies
add_subdirectory(external)

# uipc
add_subdirectory(src)

# apps
add_subdirectory(apps)