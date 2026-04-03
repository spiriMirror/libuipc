add_requires("pybind11","python")
add_requireconfs("python", "**.python", {
    override = true,
    version = get_config("python_version"),
    system = get_config("python_system"),
    configs = {
        -- https://peps.python.org/pep-0513/#libpythonx-y-so-1
        headeronly = is_plat("linux")
    }
})

target("pyuipc")
    add_rules("uipc.python")
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
    if has_config("backend_cuda") then
        add_deps("uipc_cuda_sanity_check")
    end
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
        -- local modules_target_dir = path.join(python_build_dir, "src", "uipc", "modules")
        local modules_target_dir = path.join(python_build_dir, "src", "uipc", "_native")
        
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
        if target:is_plat("windows") then
            os.cp(path.join(target_dir, "*.dll"), modules_target_dir, {
                async = true,
                detach = true,
                copy_if_different = true
            })
        elseif target:is_plat("linux") then
            os.cp(path.join(target_dir, "*.so"), modules_target_dir, {
                async = true,
                detach = true,
                copy_if_different = true
            })
        else 
            os.cp(path.join(target_dir, "*.dylib"), modules_target_dir, {
            async = true,
            detach = true,
            copy_if_different = true
        })
        end
    end)

rule("uipc.python")
    on_config(function (target)
        target:set("kind", "shared")
        target:set("prefixname", "")
        target:add("runenvs", "PYTHONPATH", target:targetdir())
        local soabi = target:extraconf("rules", "python.module", "soabi")
        if soabi == nil or soabi then
            import("lib.detect.find_tool")

            local envs = target:pkgenvs()
            local python = assert(find_tool("python3", {envs = envs}), "python not found!")
            local result = try { function() return os.iorunv(python.program, {"-c", "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"}) end}
            if result then
                result = result:trim()
                if result ~= "None" then
                    target:set("extension", result)
                end
            end
        else
            if target:is_plat("windows", "mingw") then
                target:set("extension", ".pyd")
            else
                target:set("extension", ".so")
            end
        end
    end)