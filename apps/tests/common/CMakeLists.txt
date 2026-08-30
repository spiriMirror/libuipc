file(GLOB SOURCE CONFIGURE_DEPENDS "*.cpp" "*.h")
uipc_add_test(common ${SOURCE})
target_include_directories(common PRIVATE "${PROJECT_SOURCE_DIR}/src")
