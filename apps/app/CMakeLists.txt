add_library(app STATIC)

file(GLOB SOURCE "*.cpp" "*.h")
target_sources(app PRIVATE ${SOURCE})
target_compile_definitions(app PRIVATE UIPC_ASSET_PATH="${PROJECT_SOURCE_DIR}/assets/")
target_compile_definitions(app PRIVATE UIPC_OUTPUT_PATH="${PROJECT_SOURCE_DIR}/output/")
target_link_libraries(app PRIVATE uipc)
target_include_directories(app PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")

add_library(uipc::app ALIAS app)

set_property(TARGET app PROPERTY FOLDER "apps")
