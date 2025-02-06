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
        "sanity_check"
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
