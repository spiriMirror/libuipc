enable_language(CUDA)
uipc_add_backend(cuda)

# Find CUDA Toolkit for CCCL
find_package(CUDAToolkit REQUIRED)

# basic setup
target_link_libraries(cuda PUBLIC
    muda
    cublas
    cusparse
    cusolver
    uipc_geometry
)

# Add CCCL include directory for CUDA 13.0 compatibility
if(CUDAToolkit_VERSION_MAJOR GREATER_EQUAL 13)
    target_include_directories(cuda PRIVATE ${CUDAToolkit_INCLUDE_DIRS}/cccl)
endif()

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(UIPC_GITHUB_ACTIONS)
        set(CMAKE_CUDA_ARCHITECTURES 75)
    else()
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()
endif()

target_compile_features(cuda PUBLIC cxx_std_20)
uipc_info("CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}")
target_include_directories(cuda PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES}
    CUDA_STANDARD_REQUIRED ON
    CUDA_STANDARD 20
)

# all files in this directory
file(GLOB_RECURSE SOURCES "*.cu" "*.h" "*.cpp" "*.inl")

target_sources(cuda PRIVATE ${SOURCES})

if(MSVC)
    # Disable some warnings 
    # C4819: The file contains a character that cannot be represented in the current code page
    # target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4819>)
endif()

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.cu" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})
