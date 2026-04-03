includes(
    "backends",
    "core",
    "geometry"
)

if has_config("pybind") then
    includes("pybind")
end

if has_config("grpc") then
    includes("rpc")
end

add_requires("urdfdom")

target("uipc_constitution")
    add_rules("component")
    add_files("constitution/*.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/constitution/*.h"))
    add_deps("uipc_geometry")

target("uipc_io")
    add_rules("component")
    add_files("io/*.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/io/*.h"))
    add_deps("uipc_geometry")
    add_packages("urdfdom")

target("uipc_sanity_check")
    add_rules("component")
    add_files("sanity_check/*.cpp")
    add_includedirs("sanity_check")
    add_headerfiles("sanity_check/*.h", "sanity_check/details/*.inl")
    add_deps("uipc_geometry", "uipc_io")

if has_config("backend_cuda") then
    target("uipc_cuda_sanity_check")
        add_rules("component")
        add_files("cuda_sanity_check/*.cu")
        add_includedirs("cuda_sanity_check")
        -- include cuda backend headers for InfoStacklessBVH (header-only)
        add_includedirs("backends/cuda")
        add_headerfiles("cuda_sanity_check/*.h", "cuda_sanity_check/details/*.inl")
        add_deps("uipc_geometry", "uipc_io")
        add_packages("muda")
        add_rules("cuda_warning")
        add_rules("cuda_no_host_compiler_check")
        add_cuflags("--expt-relaxed-constexpr")
        add_cuflags("--extended-lambda")
        add_links("cudart")
        if has_config("github_actions") then
            add_cugencodes("sm_89")
        else
            add_cugencodes("native")
        end
        on_load(function(target)
            if target:is_plat('windows') then
                target:add('defines', '__NV_NO_HOST_COMPILER_CHECK', {public = true})
                target:add('cuflags', '-allow-unsupported-compiler', {public = true})
                target:add('cuflags', '-Xcompiler=/wd4819', {public = true})
                target:set('toolchains', 'msvc')
            end
            target:set('toolchains', 'cuda')
        end)
end

rule("uipc_deps")
    on_load(function (target)
        local deps = {"uipc_core", "uipc_geometry", "uipc_io", "uipc_constitution", "uipc_sanity_check", "none"}
        if has_config("backend_cuda") then
            table.insert(deps, "uipc_cuda_sanity_check")
        end
        target:add("deps", deps)
    end)
