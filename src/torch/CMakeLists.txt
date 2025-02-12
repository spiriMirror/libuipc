execute_process(COMMAND ${UIPC_PYTHON_EXECUTABLE_PATH}
    "-c" "import torch; print(torch.utils.cmake_prefix_path, end='')"
    OUTPUT_VARIABLE TORCH_CMAKE_PREFIX_PATH
    RESULT_VARIABLE TORCH_CMAKE_PREFIX_PATH_RESULT
)
# convert the path to normal path: only forwards slashes
file(TO_CMAKE_PATH ${TORCH_CMAKE_PREFIX_PATH} TORCH_CMAKE_PREFIX_PATH)

if(NOT TORCH_CMAKE_PREFIX_PATH_RESULT EQUAL 0)
    uipc_error("Failed to get torch cmake prefix path.")
else()
    set(TORCH_CMAKE_PREFIX_PATH "${TORCH_CMAKE_PREFIX_PATH}/Torch")
    uipc_info("Torch cmake prefix path: ${TORCH_CMAKE_PREFIX_PATH}")
endif()

# set prefix path
set(CMAKE_PREFIX_PATH ${TORCH_CMAKE_PREFIX_PATH} ${CMAKE_PREFIX_PATH})

# find the package
find_package(Torch REQUIRED QUIET)

add_subdirectory(uipc_torch)