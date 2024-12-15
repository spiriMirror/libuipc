function uipc_add_test(name)
    target("uipc_test_" .. name)
        set_kind("binary")
        add_deps("uipc_uipc", "app_util")
        add_packages("catch2")
        add_includedirs(name, { public = true })
        add_files(name .. "/0_*.cpp")
    target_end()
end

uipc_add_test("sim_case")

