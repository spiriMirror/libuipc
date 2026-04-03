set(CMAKE_CUDA_ARCHITECTURES ${UIPC_CUDA_ARCHITECTURES})
enable_language(CUDA)

add_library(uipc_cuda_sanity_check SHARED)
add_library(uipc::cuda_sanity_check ALIAS uipc_cuda_sanity_check)

file(GLOB SOURCES "*.cu" "*.h" "details/*.inl")
target_sources(uipc_cuda_sanity_check PRIVATE ${SOURCES})
uipc_target_set_relative_source_file(uipc_cuda_sanity_check)

uipc_target_add_include_files(uipc_cuda_sanity_check)
target_link_libraries(uipc_cuda_sanity_check PUBLIC uipc::core uipc::geometry uipc::io muda::muda)
target_include_directories(uipc_cuda_sanity_check PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# include cuda backend headers for InfoStacklessBVH (header-only)
target_include_directories(uipc_cuda_sanity_check PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../backends/cuda")
uipc_target_set_output_directory(uipc_cuda_sanity_check)
uipc_target_set_rpath(uipc_cuda_sanity_check)

target_compile_features(uipc_cuda_sanity_check PUBLIC cxx_std_20)
set_target_properties(uipc_cuda_sanity_check PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES ${UIPC_CUDA_ARCHITECTURES}
    CUDA_STANDARD_REQUIRED ON
    CUDA_STANDARD 20
)

target_compile_options(uipc_cuda_sanity_check PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>: --expt-relaxed-constexpr>
)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cu" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
uipc_install_target(uipc_cuda_sanity_check)
