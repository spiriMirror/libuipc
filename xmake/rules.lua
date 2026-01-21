rule("component")
    on_load(function (target)
        target:set("kind", "shared")
        target:add("includedirs", path.join(os.projectdir(), "include"), {public = true})
        if target:is_plat("windows") then
            target:add("defines", format("%s_EXPORT_DLL", target:name():upper()))
        end
    end)
    
rule("clangd")
    on_config(function (target)
        local clangd_config_file = path.join(os.projectdir(), ".clangd")
        if os.isfile(clangd_config_file) then
            return
        end

        local cuda = import("detect.sdks.find_cuda")()
        local remove_flags = {
            "-gencode*",
            "--generate-code*",
            "-rdc=true",
            "-ccbin*",
            "--extended-lambda",
            "--expt-relaxed-constexpr",
        }

        local add_flags = {
            "-std=c++20",
            "--cuda-path=" .. path.unix(cuda.sdkdir),
        }
        for _, inc in ipairs(cuda.includedirs) do
            table.insert(add_flags, "-I" .. path.unix(inc))
        end

        local file = io.open(clangd_config_file, "w")
        if file then
            cprint("${bright}Creating clangd config file...")

            file:print("CompileFlags:")
            file:print("    Remove:")
            for _, flag in ipairs(remove_flags) do
                file:print([[        - "%s"]], flag)
            end

            file:print("    Add:")
            for _, flag in ipairs(add_flags) do
                file:print([[        - "%s"]], flag)
            end

            file:close()
        end
    end)

rule("uipc.basic")
    on_config(function (target)
        -- Add UIPC_PROJECT_DIR define
        local uipc_project_dir = path.directory(os.scriptdir())
        target:add("defines", format("UIPC_PROJECT_DIR=R\"(%s)\"", path.unix(uipc_project_dir)))

        -- -- Add UIPC_RELATIVE_SOURCE_FILE define for each source file
        -- -- Ref: https://github.com/spiriMirror/libuipc/issues/288
        for _, file in ipairs(target:sourcefiles()) do
            local rel = path.unix(file)
            local fileconfig = target:fileconfig(file) or {}
            local defines = fileconfig.defines or {}
            table.insert(defines, format("UIPC_RELATIVE_SOURCE_FILE=R\"(%s)\"", rel))
            fileconfig.defines = defines
            target:fileconfig_set(file, fileconfig)
        end
    end)