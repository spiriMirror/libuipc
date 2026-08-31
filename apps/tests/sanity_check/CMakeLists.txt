file(GLOB SOURCE CONFIGURE_DEPENDS "*.cpp" "*.h")
uipc_add_test(sanity_check ${SOURCE})
target_compile_definitions(sanity_check PRIVATE
    "UIPC_TEST_WITH_CUDA_BACKEND=$<BOOL:${UIPC_WITH_CUDA_BACKEND}>")
