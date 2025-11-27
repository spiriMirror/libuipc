if(UIPC_GITHUB_ACTIONS)
    set(CMAKE_CUDA_ARCHITECTURES 75)
else()
    set(CMAKE_CUDA_ARCHITECTURES "native")
endif()

enable_language(CUDA)

uipc_info("CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}")

# find_package(muda CONFIG REQUIRED)

# ------------------------------------------------------------------------------
# CUDA Backend
# ------------------------------------------------------------------------------

uipc_add_backend(cuda)

# basic setup
target_link_libraries(cuda PUBLIC
    muda::muda
    cublas
    cusparse
    cusolver
    uipc_geometry
)

# add CCCL include directory for CUDA 13.0 compatibility
if(CUDAToolkit_VERSION_MAJOR GREATER_EQUAL 13)
    target_include_directories(cuda PUBLIC ${CUDAToolkit_INCLUDE_DIRS}/cccl)
endif()

target_compile_features(cuda PUBLIC cxx_std_20)
target_include_directories(cuda PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES}
    CUDA_STANDARD_REQUIRED ON
    CUDA_STANDARD 20
)

target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>: --diag-suppress=27,174,997,1388,1394,1866>)

# all files in this directory
file(GLOB_RECURSE SOURCES "*.cu" "*.h" "*.hpp" "*.cpp" "*.inl")

target_sources(cuda PRIVATE ${SOURCES})

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.hpp" "*.cpp" "*.cu" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})
