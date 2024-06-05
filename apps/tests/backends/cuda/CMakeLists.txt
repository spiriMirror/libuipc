file(GLOB SOURCE "*.cpp" "*.h" "*.cu")
uipc_add_test(backend_cuda ${SOURCE})
target_link_libraries(backend_cuda PRIVATE uipc::backend::cuda)
set_target_properties(backend_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES 75
    CUDA_INCLUDE_WHAT_YOU_USE ON
)


