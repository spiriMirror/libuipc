# An empty backend, for testing purposes
uipc_add_backend(cuda)

file(GLOB SOURCES "*.cpp" "*.h" "*.cu" "details/*.inl")

target_sources(cuda PRIVATE ${SOURCES})

target_link_libraries(cuda PUBLIC muda)

set_target_properties(cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES 75
    CUDA_INCLUDE_WHAT_YOU_USE ON
)

# Group files in IDE
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCES})



if(MSVC)
    # Disable some warnings 
    # C4819: The file contains a character that cannot be represented in the current code page
    target_compile_definitions(cuda PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/W4>)
    target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CXX>:/wd4819>)
    target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4819>)
endif()
