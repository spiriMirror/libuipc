find_package(pxr CONFIG REQUIRED)
file(GLOB SOURCE "*.cpp" "*.h")
uipc_add_test(usd_support ${SOURCE})
target_link_libraries(usd_support PRIVATE
    usd
    usdGeom
    usdPhysics
)
uipc_executable_add_vcpkg_rpath(usd_support)