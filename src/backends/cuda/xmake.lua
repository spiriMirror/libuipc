rule("sail.cuda")
    on_config(function(target)
    end)
    on_load(function(target)
        target:set("cuda.rdc", false)
        local cuda_path = os.getenv("CUDA_PATH")
        if cuda_path then
            target:add("sysincludedirs", path.join(cuda_path, "include"), {public=true})
            target:add("linkdirs", path.join(cuda_path, "lib/x64/"), {public=true})
            target:add("links", "nvrtc", "cudart", "cuda", "cublas", "cusparse", "cusolver",{public=true})
        else
            target:set("enabled", false)
            return
        end
        if is_plat("windows") then
            target:add("defines", "NOMINMAX", "UNICODE")
            target:add("syslinks", "Cfgmgr32", "Advapi32")
        end
    end)

    after_build(function(target)
        local cuda_path = os.getenv("CUDA_PATH")
        if cuda_path then
            shared_files = os.match(path.join(cuda_path, "bin/*.dll"))
            -- if shared files not exists(targetdir()), copy them
            local shared_file_names = {}
            for _, shared_file in ipairs(shared_files) do
                table.insert(shared_file_names, path.filename(shared_file))
            end
            for _, shared_file_name in ipairs(shared_file_names) do
                if not os.isfile(path.join(target:targetdir(), shared_file_name)) then
                    os.cp(path.join(cuda_path, "bin", shared_file_name), target:targetdir())
                    print("copy " .. shared_file_name .. " to " .. target:targetdir())
                end
            end
            os.cp(path.join(cuda_path, "lib/x64/cudadevrt.lib"), target:targetdir())
        end
    end)
rule_end()


target("uipc_backend_cuda")
    set_kind("shared")
    add_defines("UIPC_BACKEND_EXPORT_DLL")
    add_defines("UIPC_BACKEND_DIR=\"$(targetdir)\"")
    add_defines("UIPC_BACKEND_NAME=\"cuda\"")
    add_packages("magic_enum")
    add_deps("uipc_core", "uipc_geometry", { public = true })
    add_deps("muda", { public = true })

    add_cugencodes("compute_75", {public = true})
    add_rules("sail.cuda")
    add_files("**.cu", "**.cpp")
    add_files("../common/**.cpp")
    -- add_files("../none/**.cpp")
    -- add inl
    add_includedirs(".")
    add_includedirs("../common")
    -- add_includedirs("../none/")
    -- add_defines("FMT_EXPORT")
target_end()

