file(GLOB SOURCE "*.cpp" "*.h")
uipc_add_test(sim_case ${SOURCE})
target_link_libraries(sim_case PRIVATE uipc::backends)

# Serialize PDB writes on MSVC to avoid C1041 when many .cpp files compile in parallel
if(MSVC)
    target_compile_options(sim_case PRIVATE /FS)
endif()