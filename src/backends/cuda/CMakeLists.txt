set(CMAKE_CUDA_ARCHITECTURES ${UIPC_CUDA_ARCHITECTURES})
uipc_info("CMAKE_CUDA_ARCHITECTURES: ${CMAKE_CUDA_ARCHITECTURES}.")
enable_language(CUDA)
set(UIPC_CUDA_TOOLKIT_VERSION "${CMAKE_CUDA_COMPILER_VERSION}"
    CACHE INTERNAL "CUDA toolkit version used to compile the backend" FORCE)

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
    # RDC is REQUIRED: affine_body/utils.cu defines UIPC_GENERIC free functions
    # (e.g. q_to_transform) that are called from device code in other TUs;
    # without separable compilation ptxas fails with "Unresolved extern".
    # (CMake+ninja does not rebuild objects on flag changes, so toggling this
    # needs a manual object cleanup to take effect!)
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    # Quote the list: an unquoted multi-arch list expands into separate
    # arguments and silently truncates the property to its first entry.
    CUDA_ARCHITECTURES "${UIPC_CUDA_ARCHITECTURES}"
    CUDA_STANDARD_REQUIRED ON
    CUDA_STANDARD 20
)

# CMake's Visual Studio generator does not schedule the RDC device-link when a
# target receives CUDA exclusively through OBJECT-library expressions. Give the
# final backend target a generated CUDA-language source so every generator sees
# CUDA directly. The file intentionally contains no runtime code; all 198
# functional sources remain owned by the domain OBJECT targets below.
set(UIPC_CUDA_DEVICE_LINK_ANCHOR
    "${CMAKE_CURRENT_BINARY_DIR}/uipc_cuda_device_link_anchor.cu")
file(GENERATE
    OUTPUT "${UIPC_CUDA_DEVICE_LINK_ANCHOR}"
    CONTENT "// Generated CMake CUDA-language anchor for the final RDC device-link.\n")
set_source_files_properties("${UIPC_CUDA_DEVICE_LINK_ANCHOR}" PROPERTIES GENERATED TRUE)
target_sources(cuda PRIVATE "${UIPC_CUDA_DEVICE_LINK_ANCHOR}")

target_compile_options(cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>: --diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,55,221,1028>
    $<$<COMPILE_LANGUAGE:CUDA>: --expt-relaxed-constexpr>
    $<$<COMPILE_LANGUAGE:CUDA>: --extended-lambda>
)

# Compile sources are owned by internal domain OBJECT targets and receive one
# final CUDA device-link in the public backend library.
include(${CMAKE_CURRENT_LIST_DIR}/components.cmake)

file(GLOB_RECURSE CUDA_HEADERS CONFIGURE_DEPENDS "*.h" "*.hpp" "*.inl")
target_sources(cuda PRIVATE ${CUDA_HEADERS})
uipc_target_set_relative_source_file(cuda)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES CONFIGURE_DEPENDS "*.h" "*.hpp" "*.cpp" "*.cu" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})
