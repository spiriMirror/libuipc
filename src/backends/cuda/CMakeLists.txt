set(CMAKE_CUDA_ARCHITECTURES ${UIPC_CUDA_ARCHITECTURES})
uipc_info("CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}.")
enable_language(CUDA)

# CUDA >= 13 CCCL requires the standard-conforming MSVC preprocessor for host code.
if(MSVC AND CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 13.0)
    uipc_info("CUDA >= 13 with MSVC: adding /Zc:preprocessor to CUDA host compilation.")
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>
                        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
endif()

# extended device lambdas + relaxed constexpr are required by the cuda_tool
# launch/dense math used throughout the backend.
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
                    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)

# ------------------------------------------------------------------------------
# CUDA Backend
# ------------------------------------------------------------------------------
uipc_add_backend(cuda)

# basic setup
target_link_libraries(cuda PUBLIC
    cublas
    cusparse
    cusolver
    uipc_geometry
    uipc_io
)

# add CCCL include directory for CUDA 13.0 compatibility
if(CUDAToolkit_VERSION_MAJOR GREATER_EQUAL 13)
    target_include_directories(cuda PUBLIC ${CUDAToolkit_INCLUDE_DIRS}/cccl)
endif()

target_compile_features(cuda PUBLIC cxx_std_20)
target_include_directories(cuda PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
# cuda_tool hosts the backend's device utilities (buffers/views/launch/cub/
# eigen/linear-system views); <cuda_tool/...> resolves from here.
target_include_directories(cuda PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/cuda_tool")
# the vendored muda headers reference the CUDA driver API header <cuda.h>
target_include_directories(cuda PUBLIC ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
set_target_properties(cuda PROPERTIES
    # No CUDA_SEPARABLE_COMPILATION: the backend has no cross-TU device symbols
    # (no __device__ globals, __managed__, __constant__ or texture/surface
    # objects), and RDC costs compile time in every TU plus a device-link pass.
    # (The xmake build has always compiled this target without -rdc.)
    CUDA_ARCHITECTURES ${UIPC_CUDA_ARCHITECTURES}
    CUDA_STANDARD_REQUIRED ON
    CUDA_STANDARD 20
)

target_compile_options(cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>: --diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,55,221,1028>
    $<$<COMPILE_LANGUAGE:CUDA>: --expt-relaxed-constexpr>
    $<$<COMPILE_LANGUAGE:CUDA>: --extended-lambda>
)

# all files in this directory
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "*.cu" "*.h" "*.hpp" "*.cpp" "*.inl")

target_sources(cuda PRIVATE ${SOURCES})
uipc_target_set_relative_source_file(cuda)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES CONFIGURE_DEPENDS "*.h" "*.hpp" "*.cpp" "*.cu" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})