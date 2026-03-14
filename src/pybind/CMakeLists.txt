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

# nanobind requires find_package(Python) with Interpreter and Development components
# before find_package(nanobind). Use Development.Module (CMake 3.18+) for extension modules.
set(Python_EXECUTABLE ${UIPC_PYTHON_EXECUTABLE_PATH})
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
# find the package
find_package(nanobind CONFIG REQUIRED QUIET)

add_subdirectory(pyuipc)