add_requires("pybind11")

target("pyuipc")
    add_rules("python.module")
    add_files("**.cpp")
    add_includedirs(os.scriptdir())
    add_headerfiles("**.h")

    add_deps(
        "uipc_core",
        "uipc_geometry",
        "uipc_constitution",
        "uipc_io",
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
        -- depend on backend
        if has_config("backend_cuda") then
            target:add("deps", "cuda", {inherit = false})
        end
        target:add("deps", "none", {inherit = false})
    end)

    after_build(function (target)
        local project_dir = os.projectdir()
        local python_source_dir = path.join(project_dir, "python")
        local build_dir = path.join(project_dir, "build")
        local python_build_dir = path.join(build_dir, "python")
        local modules_target_dir = path.join(python_build_dir, "src", "uipc", "modules")
        
        -- Copy the entire python folder to build directory
        print("Copying python folder from " .. python_source_dir .. " to " .. python_build_dir)
        -- os.cp copies directories recursively when given a directory path
        os.mkdir(modules_target_dir)
        os.cp(python_source_dir, python_build_dir, {
            async = true,
            detach = true,
            copy_if_different = true
        })
        os.cp(python_source_dir, python_build_dir, {
            async = true,
            detach = true,
            copy_if_different = true
        })
        
        -- Ensure modules directory exists
        
        -- Copy the built modules from target directory to build_dir/python/src/uipc/modules/
        local target_dir = target:targetdir()
        print("Copying modules from " .. target_dir .. " to " .. modules_target_dir)
        
        -- Copy all files from target directory to modules directory
        -- Use os.cp with pattern to copy all files
        os.cp(path.join(target_dir, "*.dll"), modules_target_dir, {
            async = true,
            detach = true,
            copy_if_different = true
        })
        os.cp(path.join(target_dir, "*.pyd"), modules_target_dir, {
            async = true,
            detach = true,
            copy_if_different = true
        })
    end)