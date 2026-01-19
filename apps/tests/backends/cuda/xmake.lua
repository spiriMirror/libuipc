target("backend_cuda")
    set_group("tests")
    add_rules("uipc_test")
    add_deps("cuda")
    add_files("**.cu")
    if has_config("dev") then
        add_rules("clangd")
    end
    if has_config("github_actions") then
        add_cugencodes("sm_89")
    else
        add_cugencodes("native")
    end
    add_cuflags("/wd4819", {tools = "cl"})
    add_packages("muda")