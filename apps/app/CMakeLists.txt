add_library(app STATIC)

target_compile_definitions(app PRIVATE UIPC_ASSET_PATH="${PROJECT_SOURCE_DIR}/assets/")
target_compile_definitions(app PRIVATE UIPC_OUTPUT_PATH="${PROJECT_SOURCE_DIR}/output/")
target_link_libraries(app PUBLIC uipc::uipc)
target_include_directories(app PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")

set_property(TARGET app PROPERTY FOLDER "apps")

file(GLOB SOURCE "*.cpp" "*.h")
target_sources(app PRIVATE ${SOURCE})