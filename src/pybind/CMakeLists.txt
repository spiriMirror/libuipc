if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Building pyuipc requires an out-of-source build directory (cmake -S . -B build).")
endif()
uipc_require_python_module(${UIPC_PYTHON_EXECUTABLE_PATH} "pybind11" 2.12)

execute_process(COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH}
    "-c" 
    "import pybind11; print(pybind11.get_cmake_dir(), end='')"
    OUTPUT_VARIABLE PYBIND11_CMAKE_PREFIX_PATH
    RESULT_VARIABLE PYBIND11_CMAKE_PREFIX_PATH_RESULT
)
# convert the path to normal path: only forwards slashes
file(TO_CMAKE_PATH ${PYBIND11_CMAKE_PREFIX_PATH} PYBIND11_CMAKE_PREFIX_PATH)
set(PYBIND11_CMAKE_PREFIX_PATH ${PYBIND11_CMAKE_PREFIX_PATH})
uipc_info("Pybind11 cmake prefix path: ${PYBIND11_CMAKE_PREFIX_PATH}")
set(CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_PREFIX_PATH};${CMAKE_PREFIX_PATH}")

set(PYBIND11_FINDPYTHON ON)
# find python for pybind11
# specify the python executable to use
set(Python3_EXECUTABLE ${UIPC_PYTHON_EXECUTABLE_PATH})
# find the package
find_package(pybind11 CONFIG REQUIRED QUIET)
if(pybind11_VERSION VERSION_LESS 2.12)
    message(FATAL_ERROR "pyuipc requires pybind11 2.12 or newer (found ${pybind11_VERSION}).")
endif()

add_subdirectory(pyuipc)
