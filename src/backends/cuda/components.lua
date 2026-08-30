rule("cuda.component")
    on_load(function (target)
        import("core.base.semver")

        local version = semver.new(target:version())
        target:set("kind", "object")
        target:set("default", false)
        target:set("group", "uipc-backends/cuda-components")
        target:add("includedirs",
            path.directory(os.scriptdir()),
            os.scriptdir(),
            path.join(os.scriptdir(), "cuda_tool"))
        target:add("defines",
            "UIPC_BACKEND_EXPORT_DLL",
            [[UIPC_BACKEND_NAME=R"(cuda)"]],
            "UIPC_VERSION_MAJOR=" .. version:major(),
            "UIPC_VERSION_MINOR=" .. version:minor(),
            "UIPC_VERSION_PATCH=" .. version:patch())
        target:add("deps", "uipc_core", "uipc_geometry", "uipc_io")
        target:add("cuflags", "--expt-relaxed-constexpr", "--extended-lambda", "-rdc=true")
        if has_config("github_actions") then
            target:add("cugencodes", "sm_89")
        else
            target:add("cugencodes", "native")
        end
        if target:is_plat("windows") then
            target:add("defines", "__NV_NO_HOST_COMPILER_CHECK")
            target:add("cuflags",
                "-allow-unsupported-compiler",
                "-Xcompiler=/wd4819",
                "-Xcompiler=/Zc:preprocessor")
            target:set("toolchains", "msvc")
        end
        target:set("toolchains", "cuda")
    end)
rule_end()

local components = {
    runtime = {
        "*.cpp",
        "animator/**.cu",
        "diff_sim/**.cu",
        "engine/**.cu",
        "external_force/**.cu",
        "global_geometry/**.cu",
        "implicit_geometry/**.cu",
        "joint_dof_system/**.cu",
        "line_search/**.cu",
        "newton_tolerance/**.cu",
        "pipeline/**.cu",
        "sanity_check/**.cu",
        "time_integrator/**.cu",
        "utils/**.cpp"
    },
    affine_body = {"affine_body/**.cu"},
    collision = {
        "collision_detection/*.cu",
        "collision_detection/filters/al_vertex_half_plane_trajectory_filter.cu",
        "collision_detection/filters/easy_vertex_half_plane_trajectory_filter.cu",
        "collision_detection/filters/info_stackless_bvh_simplex_trajectory_filter.cu"
    },
    collision_legacy = {
        "collision_detection/filters/info_stackless_bvh_v0_simplex_trajectory_filter.cu",
        "collision_detection/filters/lbvh_simplex_trajectory_filter.cu",
        "collision_detection/filters/stackless_bvh_simplex_trajectory_filter.cu"
    },
    contact = {
        "active_set_system/**.cu",
        "contact_system/**.cu",
        "distance_system/**.cu",
        "dytopo_effect_system/**.cu",
        "inter_primitive_effect_system/**.cu"
    },
    fem = {"finite_element/**.cu"},
    linear_system = {"linear_system/**.cu"},
    coupling = {"coupling_system/**.cu"}
}

local component_targets = {}
local assigned_sources = {}
for name, patterns in pairs(components) do
    local target_name = "cuda_" .. name .. "_objects"
    local enabled = name ~= "collision_legacy" or has_config("cuda_legacy_collision")
    if enabled then
        table.insert(component_targets, target_name)
        target(target_name)
            add_rules("cuda.component", "cuda_warning", "cuda_no_host_compiler_check")
            add_files(table.unpack(patterns))
        target_end()
    end

    for _, pattern in ipairs(patterns) do
        for _, file in ipairs(os.files(path.join(os.scriptdir(), pattern))) do
            local absolute = path.absolute(file)
            if assigned_sources[absolute] then
                raise("CUDA source belongs to multiple components: " .. absolute)
            end
            assigned_sources[absolute] = target_name
        end
    end
end

for _, extension in ipairs({"**.cpp", "**.cu"}) do
    for _, file in ipairs(os.files(path.join(os.scriptdir(), extension))) do
        local absolute = path.absolute(file)
        if not assigned_sources[absolute] then
            raise("CUDA source has no internal component: " .. absolute)
        end
    end
end

table.sort(component_targets)
return component_targets
