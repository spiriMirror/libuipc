include(uipc_utils)
add_executable(hello_uipc)

target_sources(hello_uipc PRIVATE "main.cpp")
target_link_libraries(hello_uipc PRIVATE uipc)

uipc_set_output_directory(hello_uipc)

set_property(TARGET hello_uipc PROPERTY FOLDER "apps/examples")