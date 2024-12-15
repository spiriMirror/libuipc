find_package(Python REQUIRED QUIET)
find_package (Python COMPONENTS Interpreter Development REQUIRED QUIET)

if(NOT Python_FOUND)
    uipc_error("Python is required to generate vcpkg.json. Please install Python.")
endif()

uipc_require_python_module(${Python_EXECUTABLE} "pybind11")

execute_process(COMMAND ${Python_EXECUTABLE}
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

# find the package
find_package(pybind11 CONFIG REQUIRED QUIET)

add_subdirectory(pyuipc)