add_library(uipc_constitution SHARED)
add_library(uipc::constitution ALIAS uipc_constitution)

uipc_target_add_include_files(uipc_constitution)
target_link_libraries(uipc_constitution PUBLIC uipc::core uipc::geometry)
target_compile_definitions(uipc_constitution PRIVATE UIPC_CONSTITUTION_EXPORT_DLL=1) # export dll

uipc_target_set_output_directory(uipc_constitution)

file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_constitution PRIVATE ${SOURCES})

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})
