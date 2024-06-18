include(uipc_utils)
add_executable(hello_affine_body)

target_sources(hello_affine_body PRIVATE "main.cpp")

target_link_libraries(hello_affine_body PRIVATE uipc::uipc uipc::app)

uipc_set_output_directory(hello_affine_body)
uipc_add_backend_dependency(hello_affine_body)

set_property(TARGET hello_affine_body PROPERTY FOLDER "apps/examples")