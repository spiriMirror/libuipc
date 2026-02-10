# An empty backend, for testing purposes
uipc_add_backend(none)

target_include_directories(none PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")

target_sources(none PRIVATE ${SOURCES})
uipc_target_set_relative_source_file(none)

# Group files in IDE
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/.." FILES ${SOURCES})
