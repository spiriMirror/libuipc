file(GLOB SOURCE "*.cpp" "*.h")
uipc_add_test(regression ${SOURCE})
target_link_libraries(regression PRIVATE uipc::backends)

# Serialize PDB writes on MSVC to avoid C1041 when many .cpp files compile in parallel
if(MSVC)
    target_compile_options(regression PRIVATE /FS)
endif()
