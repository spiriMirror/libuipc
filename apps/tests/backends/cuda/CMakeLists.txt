enable_language(CUDA)

# CUDA >= 13 CCCL requires the standard-conforming MSVC preprocessor for host code.
if(MSVC AND CMAKE_CUDA_COMPILER_VERSION VERSION_GREATER_EQUAL 13.0)
    uipc_info("CUDA >= 13 with MSVC: adding /Zc:preprocessor to CUDA host compilation.")
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:/Zc:preprocessor>
                        $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/Zc:preprocessor>)
endif()

# extended device lambdas + relaxed constexpr are required by the cuda_tool
# launch/dense math used throughout the backend (and fmt's constexpr literals).
add_compile_options($<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>
                    $<$<COMPILE_LANGUAGE:CUDA>:--expt-relaxed-constexpr>)

file(GLOB_RECURSE SOURCE "*.cpp" "*.h" "*.cu" ".inl")
uipc_add_test(backend_cuda ${SOURCE})
target_link_libraries(backend_cuda PRIVATE uipc::backend::cuda)
target_compile_options(backend_cuda PRIVATE
    $<$<COMPILE_LANGUAGE:CUDA>: --diag-suppress=20012,1388,27,174,1394,997,1866,69,177,554,20014,2361,20011,940,55,221,1028>)

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(UIPC_GITHUB_ACTIONS)
        set(CMAKE_CUDA_ARCHITECTURES 75)
    else()
        set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()
endif()

set_target_properties(backend_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES ${CMAKE_CUDA_ARCHITECTURES}
    CUDA_INCLUDE_WHAT_YOU_USE ON
)
