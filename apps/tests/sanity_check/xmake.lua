target("sanity_check")
    set_group("tests")
    add_rules("uipc_test")
    add_files("*.cpp")
    if has_config("backend_cuda") then
        add_defines("UIPC_TEST_WITH_CUDA_BACKEND=1")
    else
        add_defines("UIPC_TEST_WITH_CUDA_BACKEND=0")
    end
