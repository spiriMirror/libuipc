includes("components.lua")
local component_targets = {
    "cuda_affine_body_objects",
    "cuda_collision_objects",
    "cuda_contact_objects",
    "cuda_coupling_objects",
    "cuda_fem_objects",
    "cuda_linear_system_objects",
    "cuda_runtime_objects"
}
if has_config("cuda_legacy_collision") then
    table.insert(component_targets, "cuda_collision_legacy_objects")
end

target("cuda")
    add_rules("backend")
    if has_config("dev") then
        add_rules("clangd")
    end
    add_headerfiles("**.h", "**.inl")
    add_deps(table.unpack(component_targets))
    add_includedirs(os.scriptdir(), {public = true})
    -- cuda_tool (self-written device utilities) on the include path for <cuda_tool/...>
    add_includedirs(path.join(os.scriptdir(), "cuda_tool"), {public = true})
    if has_config("github_actions") then
        add_cugencodes("sm_89")
    else
        add_cugencodes("native")
    end
    add_rules("cuda_warning")
    add_rules("cuda_no_host_compiler_check")    
    add_cuflags("--expt-relaxed-constexpr")
    add_cuflags("--extended-lambda")
    -- RDC is required: affine_body/utils.cu defines UIPC_GENERIC free functions
    -- (e.g. q_to_transform) called from device code in other TUs; without
    -- separable compilation ptxas fails with "Unresolved extern".
    -- (parity with CUDA_SEPARABLE_COMPILATION ON in CMakeLists.txt)
    add_cuflags("-rdc=true")
    
    add_links(
        "cudart",
        "cublas",
        "cusparse",
        "cusolver"
    )

    add_deps("uipc_geometry", "uipc_io")
    on_load(function(target)
        if target:is_plat('windows') then
            target:add('defines', '__NV_NO_HOST_COMPILER_CHECK', {public = true})
            target:add('cuflags', '-allow-unsupported-compiler', {public = true})
            -- Suppress MSVC C4819 for host compilation of CUDA sources.
            target:add('cuflags', '-Xcompiler=/wd4819', {public = true})
            -- CUDA >= 13 CCCL requires the standard-conforming MSVC preprocessor
            -- (parity with src/backends/cuda/CMakeLists.txt).
            target:add('cuflags', '-Xcompiler=/Zc:preprocessor', {public = true})
            target:set('toolchains', 'msvc')
        end
        target:set('toolchains', 'cuda')
    end)
