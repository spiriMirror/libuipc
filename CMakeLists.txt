cmake_minimum_required(VERSION 3.26)
# set cmake include path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(cmake/uipc_utils.cmake)

# =========================================================================
# Project Version
# =========================================================================
if(DEFINED SKBUILD_PROJECT_VERSION)
   set(UIPC_VERSION ${SKBUILD_PROJECT_VERSION})
else()
    set(UIPC_VERSION "0.9.0")
endif()
uipc_parse_version(${UIPC_VERSION} UIPC_VERSION_MAJOR UIPC_VERSION_MINOR UIPC_VERSION_PATCH)

# =========================================================================
# Project Options
# =========================================================================
option(UIPC_USING_LOCAL_VCPKG "Using local vcpkg" ON)
option(UIPC_BUILD_GUI "Build GUI, turn it off if you're server only" OFF)
option(UIPC_BUILD_PYBIND "Build pyuipc" OFF)
option(UIPC_BUILD_PYTHON_WHEEL "Wheel build mode for Python post-build helper" OFF)
option(UIPC_BUILD_EXAMPLES "Build examples" ON)
option(UIPC_BUILD_TESTS "Build tests" ON)
option(UIPC_BUILD_BENCHMARKS "Build benchmarks" ON)
option(UIPC_DEV_MODE "Enable developer mode" OFF)
option(UIPC_WITH_USD_SUPPORT "Build with Universal Scene Description (USD) support" OFF)
option(UIPC_WITH_VDB_SUPPORT "Build with Volumetric DataBase (VDB) support" OFF)

set(UIPC_PYTHON_EXECUTABLE_PATH "" CACHE PATH "Python executable path")
set(UIPC_USD_INSTALL_DIR "" CACHE PATH
        "USD installation directory: the folder that contains pxrConfig.cmake, e.g. ~/Software/Usd")

# Backend Options
# =========================================================================
# if MAC OS, disable CUDA backend
if(APPLE)
    set(CUDA_BACKEND_ON OFF)
else()
    set(CUDA_BACKEND_ON ON)
endif()

option(UIPC_WITH_CUDA_BACKEND "Build with CUDA backend" ${CUDA_BACKEND_ON})
set(UIPC_CUDA_ARCHITECTURES "native" CACHE STRING "Setup CUDA architectures (e.g. \"75,89\" or \"native\")")

# Install Options
# =========================================================================
set(UIPC_INSTALL_DIR "." CACHE PATH "Install directory, default is current directory")
if(DEFINED SKBUILD)
    if(NOT SKBUILD EQUAL 2)
        uipc_error("MUST use scikit-build-core(2), but got ${SKBUILD}")
    endif()
    set(UIPC_INSTALL_DIR "uipc/_native")
    set(UIPC_BUILD_PYTHON_WHEEL ON)
endif()

if(UIPC_BUILD_PYTHON_WHEEL)
    set(UIPC_BUILD_PYBIND ON)
endif()

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

project(uipc VERSION ${UIPC_VERSION})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json

# dependencies
add_subdirectory(external)

# uipc
add_subdirectory(src)

# apps
add_subdirectory(apps)

# install headers (libuipc/include)
uipc_install_headers()