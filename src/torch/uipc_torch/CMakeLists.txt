add_library(uipc_torch SHARED)
target_compile_features(uipc_torch PUBLIC cxx_std_20)
target_link_libraries(uipc_torch PUBLIC ${TORCH_LIBRARIES})
target_link_libraries(uipc_torch PUBLIC uipc::uipc)
target_include_directories(uipc_torch PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../")
uipc_target_add_include_files(uipc_torch)
uipc_target_set_output_directory(uipc_torch)

file(GLOB SOURCES "*.cpp" "*.h")
target_sources(uipc_torch PRIVATE ${SOURCES})

# Setup IDE
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCES})