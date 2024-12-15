include(uipc_utils)

set(app_name hello_affine_body)

add_executable(${app_name})
target_sources(${app_name} PRIVATE "main.cpp")
target_link_libraries(${app_name} PRIVATE uipc::uipc app)

uipc_target_set_output_directory(${app_name})
uipc_target_add_backend_dependency(${app_name})
set_property(TARGET ${app_name} PROPERTY FOLDER "apps/examples")