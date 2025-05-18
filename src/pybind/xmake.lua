add_requires("pybind11")

target("pyuipc")
    add_rules("python.library")
    add_files("**.cpp")
    add_includedirs(os.scriptdir())
    add_headerfiles("**.h")

    add_deps(
        "core",
        "geometry",
        "constitution",
        "io",
        "uipc_sanity_check"
    )
    add_packages("pybind11")

    on_load(function (target)
        import("core.base.semver")

        if target:get("version") then
            local version = semver.new(target:get("version"))
            target:add("defines",
                "UIPC_VERSION_MAJOR=" .. version:major(),
                "UIPC_VERSION_MINOR=" .. version:minor(),
                "UIPC_VERSION_PATCH=" .. version:patch()
            )
        end
    end)

    after_build(function (target)
        local py_module_target = "$(projectdir)" .. "/python/src/uipc/modules/" 
        -- make module target if it doesn't exist
        os.mkdir(py_module_target)
        print("Copying folder from " .. target:targetdir() .. " to " .. py_module_target)
        os.cp(target:targetdir(), py_module_target)
    end)