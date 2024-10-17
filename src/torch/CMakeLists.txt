find_package(Python REQUIRED QUIET)
if(NOT Python_FOUND)
    uipc_error("Python is required to generate vcpkg.json. Please install Python.")
endif()
execute_process(COMMAND ${Python_EXECUTABLE}
    "-c" "import torch; print(torch.utils.cmake_prefix_path, end='')"
    OUTPUT_VARIABLE TORCH_CMAKE_PREFIX_PATH
    RESULT_VARIABLE TORCH_CMAKE_PREFIX_PATH_RESULT
)

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