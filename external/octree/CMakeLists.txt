uipc_init_submodule(Octree)
add_library(octree INTERFACE)
target_include_directories(octree INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(octree PROPERTIES FOLDER "external")