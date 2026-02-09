find_package(libigl REQUIRED)
find_package(TBB REQUIRED)
find_package(Octree CONFIG REQUIRED)

add_library(uipc_geometry SHARED)
add_library(uipc::geometry ALIAS uipc_geometry)

target_include_directories(uipc_geometry PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

uipc_target_add_include_files(uipc_geometry)

target_link_libraries(uipc_geometry PRIVATE 
    uipc::core
    igl::igl_core
    Octree::Octree
    TBB::tbb
)

if(UIPC_WITH_VDB_SUPPORT)
    target_link_libraries(uipc_geometry PRIVATE uipc::vdb)
endif()

target_compile_definitions(uipc_geometry PRIVATE UIPC_GEOMETRY_EXPORT_DLL=1) # export dll

file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_geometry PRIVATE ${SOURCES})

uipc_target_set_output_directory(uipc_geometry)
uipc_target_set_rpath(uipc_geometry)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})

add_subdirectory(implicit_geometries)
add_subdirectory(affine_body)
add_subdirectory(bvh)
add_subdirectory(graph_coloring)

uipc_target_set_relative_source_file(uipc_geometry)

# ------------------------------------------------------------------------------
# Install targets
# ------------------------------------------------------------------------------
uipc_install_target(uipc_geometry)