uipc_require_python_module(${UIPC_PYTHON_EXECUTABLE_PATH} "nanobind")

execute_process(COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH}
    "-c" 
    "import nanobind; print(nanobind.cmake_dir(), end='')"
    OUTPUT_VARIABLE NANOBIND_CMAKE_DIR
    RESULT_VARIABLE NANOBIND_CMAKE_DIR_RESULT
)
# convert the path to normal path: only forwards slashes
file(TO_CMAKE_PATH ${NANOBIND_CMAKE_DIR} NANOBIND_CMAKE_DIR)
uipc_info("Nanobind cmake dir: ${NANOBIND_CMAKE_DIR}")
set(CMAKE_PREFIX_PATH "${NANOBIND_CMAKE_DIR};${CMAKE_PREFIX_PATH}")

# find python for nanobind
# specify the python executable to use
set(Python3_EXECUTABLE ${UIPC_PYTHON_EXECUTABLE_PATH})
# find the package
find_package(nanobind CONFIG REQUIRED QUIET)

add_subdirectory(pyuipc)