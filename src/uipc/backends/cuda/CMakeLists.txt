uipc_add_backend(cuda)

# basic setup
target_link_libraries(cuda PUBLIC
    muda
    cublas
    cusparse
    cusolver
)
target_compile_features(cuda PUBLIC cxx_std_20)
target_include_directories(cuda PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    CUDA_ARCHITECTURES 75
    CUDA_INCLUDE_WHAT_YOU_USE ON
)

# add subdirectories
add_subdirectory(collision_detection)
add_subdirectory(affine_body)
add_subdirectory(linear_system)
add_subdirectory(algorithm)
add_subdirectory(line_search)
add_subdirectory(contact_system)
add_subdirectory(global_geometry)
add_subdirectory(engine)

# source files in this directory
file(GLOB SOURCES "*.cpp" "*.h" "*.cu" "details/*.inl")
target_sources(cuda PRIVATE ${SOURCES})

if(MSVC)
    # Disable some warnings 
    # C4819: The file contains a character that cannot be represented in the current code page
    target_compile_options(cuda PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=/wd4819>)
endif()

# Setup IDE
get_target_property(SOURCES cuda SOURCES)
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../" FILES ${SOURCES})
