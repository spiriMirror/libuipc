file(GLOB SOURCE "*.cpp" "*.h")
uipc_add_test(sim_case ${SOURCE})
target_link_libraries(sim_case PRIVATE uipc::backends)