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
# ===============================================================================

option(UIPC_USING_LOCAL_VCPKG "Using local vcpkg" ON)
option(UIPC_BUILD_GUI "Build GUI, turn it off if you're server only" OFF)
option(UIPC_BUILD_PYBIND "Build pyuipc" ON)
option(UIPC_BUILD_EXAMPLES "Build examples" OFF)
option(UIPC_BUILD_TESTS "Build tests" OFF)
option(UIPC_BUILD_BENCHMARKS "Build benchmarks" OFF)
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

# Suppress warnings on Windows
if(WIN32 AND MSVC)
    # For CUDA files, use CUDA-specific flags to avoid conflicts
    if(CMAKE_CUDA_COMPILER)
        # CUDA warnings
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --diag-suppress=1394")
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --diag-suppress=20012")
        # Pass MSVC flags through CUDA compiler only
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/W1")
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/wd4068")
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/wd4251")
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/wd4275")
        set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/wd4996")
    endif()

    # For non-CUDA C++ files, use regular MSVC flags
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W1 /wd4068 /wd4251 /wd4275 /wd4996")
endif()

# dependencies
add_subdirectory(external)

# uipc
add_subdirectory(src)

# apps
add_subdirectory(apps)