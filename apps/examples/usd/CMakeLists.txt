include(uipc_utils)

set(app_name usd_app)

add_executable(${app_name})
target_sources(${app_name} PRIVATE "main.cpp")

uipc_target_set_output_directory(${app_name})

find_package(pxr REQUIRED CONFIG)
target_compile_features(${app_name} PUBLIC cxx_std_20)

target_link_libraries(${app_name} PRIVATE 
    usd
    usdGeom
    )

uipc_executable_add_vcpkg_rpath(${app_name})