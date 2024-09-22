add_library(uipc_io SHARED)
add_library(uipc::io ALIAS uipc_io)

uipc_target_add_include_files(uipc_io)
target_link_libraries(uipc_io PUBLIC uipc::core uipc::geometry)
target_compile_definitions(uipc_io PRIVATE UIPC_IO_EXPORT_DLL=1) # export dll

file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_io PRIVATE ${SOURCES})

uipc_target_set_output_directory(uipc_io)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})