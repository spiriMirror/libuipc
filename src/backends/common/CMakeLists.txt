add_library(backend_common STATIC)

file(GLOB_RECURSE SOURCES "*.cpp" "*.c" "*.h")
target_include_directories(backend_common PUBLIC "${PROJECT_SOURCE_DIR}/src")
target_sources(backend_common PRIVATE ${SOURCES})
target_link_libraries(backend_common PUBLIC uipc::core)
set_target_properties(backend_common PROPERTIES FOLDER "uipc-backends")