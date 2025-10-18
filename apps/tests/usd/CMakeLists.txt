file(GLOB SOURCE "*.cpp" "*.h")
uipc_add_test(usd_support ${SOURCE})
uipc_executable_add_vcpkg_rpath(usd_support)