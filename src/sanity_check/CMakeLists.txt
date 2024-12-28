add_library(uipc_sanity_check SHARED)
add_library(uipc::sanity_check ALIAS uipc_sanity_check)
file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(uipc_sanity_check PRIVATE ${SOURCES})

uipc_target_add_include_files(uipc_sanity_check)
target_link_libraries(uipc_sanity_check PUBLIC uipc::core uipc::geometry uipc::io)
target_include_directories(uipc_sanity_check PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
uipc_target_set_output_directory(uipc_sanity_check)

# ------------------------------------------------------------------------------
# setup source group for the IDE
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SOURCE_GROUP_FILES "*.h" "*.cpp" "*.inl")
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCE_GROUP_FILES})