find_package(libigl REQUIRED)


add_library(uipc_geometry SHARED)
add_library(uipc::geometry ALIAS uipc_geometry)

target_include_directories(uipc_geometry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

uipc_target_add_include_files(uipc_geometry)

target_link_libraries(uipc_geometry PUBLIC 
    uipc::core
    igl::igl_core
    tetgen
    octree)

target_compile_definitions(uipc_geometry PRIVATE UIPC_GEOMETRY_EXPORT_DLL=1) # export dll

file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_geometry PRIVATE ${SOURCES})

uipc_target_set_output_directory(uipc_geometry)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})

add_subdirectory(implicit_geometries)
add_subdirectory(affine_body)
add_subdirectory(bvh)