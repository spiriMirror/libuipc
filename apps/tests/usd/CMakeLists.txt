find_package(pxr CONFIG REQUIRED)
file(GLOB SOURCE "*.cpp" "*.h")
add_executable(usd_support ${SOURCE})
# Link1: Must Link usd before uipc::core to avoid tbb header file conflict
target_link_libraries(usd_support PRIVATE
        usd
        usdGeom
        usdPhysics
)
# Link2: Must Link uipc::core after usd to avoid tbb header file conflict (in uipc_executable_as_test)
uipc_executable_as_test(usd_support)
uipc_executable_add_vcpkg_rpath(usd_support)