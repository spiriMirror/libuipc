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

local component_order = {
    "runtime",
    "affine_body",
    "collision",
    "collision_legacy",
    "contact",
    "fem",
    "linear_system",
    "coupling"
}
local backend_dir = os.scriptdir()

-- XMake's built-in CUDA device-link only scans CUDA source batches owned by
-- the final binary/shared target. Keep the ownership partition as a manifest,
-- but attach its sources directly to the one shared backend at project-parse
-- time so every RDC object participates in the single device-link on both
-- Windows and Linux.
function uipc_add_cuda_component_sources()
    for _, name in ipairs(component_order) do
        if name ~= "collision_legacy" or has_config("cuda_legacy_collision") then
            for _, pattern in ipairs(components[name]) do
                add_files(path.join(backend_dir, pattern))
            end
        end
    end
end

local assigned_sources = {}
for _, name in ipairs(component_order) do
    for _, pattern in ipairs(components[name]) do
        for _, file in ipairs(os.files(path.join(backend_dir, pattern))) do
            local absolute = path.absolute(file)
            if assigned_sources[absolute] then
                raise("CUDA source belongs to multiple components: " .. absolute)
            end
            assigned_sources[absolute] = name
        end
    end
end

for _, extension in ipairs({"**.cpp", "**.cu"}) do
    for _, file in ipairs(os.files(path.join(backend_dir, extension))) do
        local absolute = path.absolute(file)
        if not assigned_sources[absolute] then
            raise("CUDA source has no internal component: " .. absolute)
        end
    end
end
