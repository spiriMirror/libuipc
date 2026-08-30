#include "scene_default_config.h"
#include <uipc/common/unit.h>
#include <cmath>

namespace uipc::core
{
struct SceneConfigContract
{
    geometry::AttributeCollection config;
    Json                          entries = Json::object();
};

static SceneConfigContract make_scene_config_contract()
{
    SceneConfigContract contract;
    contract.config.resize(1);
    auto& entries = contract.entries;

    auto add = [&](std::string_view                        key,
                   auto                                    default_value,
                   std::string_view                        type,
                   std::string_view                        description,
                   std::initializer_list<std::string_view> consumers,
                   Json             constraints = Json::object(),
                   std::string_view unit        = {})
    {
        const std::string key_string{key};
        UIPC_ASSERT_THROW(!entries.contains(key_string),
                          "Duplicate scene config metadata for '{}'.",
                          key);

        contract.config.create(key_string, default_value);
        auto attribute = contract.config.find(key_string);
        UIPC_ASSERT_THROW(attribute, "Scene config attribute '{}' was not created.", key);

        Json entry{{"type", std::string{type}},
                   {"description", std::string{description}},
                   {"lifecycle", "set before World::init(scene)"},
                   {"status", "active"},
                   {"consumers", Json::array()}};
        entry["default"]     = attribute->to_json(0);
        entry["storageType"] = std::string{attribute->type_name()};
        if(!unit.empty())
            entry["unit"] = std::string{unit};
        if(type == "number" || type == "array")
            entry["finite"] = true;
        for(auto consumer : consumers)
            entry["consumers"].push_back(std::string{consumer});
        for(auto&& [name, value] : constraints.items())
            entry[name] = value;
        entries[key_string] = std::move(entry);
    };

    const Json flag{{"enum", Json::array({0, 1})}};

    add("dt",
        Float{0.01},
        "number",
        "Time represented by one simulation frame.",
        {"src/backends/cuda/engine", "src/core/core/internal/scene.cpp"},
        Json{{"exclusiveMinimum", 0.0}},
        "s");
    add("gravity",
        Vector3{0.0, -9.8, 0.0},
        "array",
        "Global acceleration as a three-component column vector.",
        {"src/backends/cuda/finite_element", "src/backends/cuda/affine_body"},
        Json{{"semanticType", "Vector3"}, {"componentCount", 3}},
        "m/s^2");
    add("cfl/enable",
        IndexT{0},
        "integer",
        "Enable the experimental contact CFL step filter.",
        {"src/backends/cuda/contact_system"},
        flag);
    add("integrator/type",
        std::string{"bdf1"},
        "string",
        "Select the backward-differentiation time integrator.",
        {"src/backends/cuda/time_integrator"},
        Json{{"enum", Json::array({"bdf1", "bdf2"})}});

    add("newton/max_iter",
        IndexT{1024},
        "integer",
        "Maximum nonlinear iterations per frame.",
        {"src/backends/cuda/engine"},
        Json{{"minimum", 1}});
    add("newton/min_iter",
        IndexT{0},
        "integer",
        "Hard floor before ordinary Newton convergence may terminate.",
        {"src/backends/cuda/engine"},
        Json{{"minimum", 0}});
    add("newton/use_adaptive_tol",
        IndexT{0},
        "integer",
        "Reserved compatibility key; adaptive tolerance has no implemented consumer.",
        {},
        Json{{"const", 0}});
    entries["newton/use_adaptive_tol"]["status"] = "reserved";
    add("newton/velocity_tol",
        Float{0.05_m / 1.0_s},
        "number",
        "Absolute velocity tolerance used by the displacement convergence test.",
        {"src/backends/cuda/newton_tolerance/max_translation_checker.cu"},
        Json{{"exclusiveMinimum", 0.0}},
        "m/s");
    add("newton/velocity_tol_relative",
        Float{0.0},
        "number",
        "Positive values replace the absolute tolerance with this fraction of the rest-scene diagonal.",
        {"src/backends/cuda/newton_tolerance/max_translation_checker.cu"});
    add("newton/ccd_tol",
        Float{1.0},
        "number",
        "Minimum accepted CCD step fraction for Newton convergence.",
        {"src/backends/cuda/newton_tolerance"},
        Json{{"exclusiveMinimum", 0.0}, {"maximum", 1.0}});
    add("newton/transrate_tol",
        Float{0.1 / 1.0_s},
        "number",
        "Affine-body transform-rate convergence tolerance.",
        {"src/backends/cuda/newton_tolerance"},
        Json{{"minimum", 0.0}},
        "1/s");
    add("newton/semi_implicit/enable",
        IndexT{0},
        "integer",
        "Enable semi-implicit beta termination.",
        {"src/backends/cuda/engine"},
        flag);
    add("newton/semi_implicit/beta_tol",
        Float{1e-3},
        "number",
        "Early-exit threshold for accumulated semi-implicit beta.",
        {"src/backends/cuda/engine"},
        Json{{"minimum", 0.0}, {"maximum", 1.0}});
    add("newton/semi_implicit/K_min",
        IndexT{1},
        "integer",
        "Newton iteration at which semi-implicit beta accumulation starts.",
        {"src/backends/cuda/engine"},
        Json{{"minimum", 0}});

    add("linear_system/tol_rate",
        Float{1e-3},
        "number",
        "Relative PCG residual tolerance.",
        {"src/backends/cuda/linear_system"},
        Json{{"exclusiveMinimum", 0.0}, {"exclusiveMaximum", 1.0}});
    add("linear_system/solver",
        std::string{"fused_pcg"},
        "string",
        "Select the global iterative linear solver.",
        {"src/backends/cuda/linear_system"},
        Json{{"enum", Json::array({"fused_pcg", "linear_pcg"})}});
    add("linear_system/fem_preconditioner",
        std::string{"diag"},
        "string",
        "Select diagonal block-Jacobi or Multi-Level Additive Schwarz for FEM.",
        {"src/backends/cuda/finite_element"},
        Json{{"enum", Json::array({"diag", "mas"})}});
    add("linear_system/use_cuda_graph",
        IndexT{1},
        "integer",
        "Fused-PCG launch mode: plain, block replay, or a device-side loop.",
        {"src/backends/cuda/linear_system/linear_fused_pcg.cu"},
        Json{{"enum", Json::array({0, 1, 2})}});
    add("linear_system/check_interval",
        IndexT{5},
        "integer",
        "PCG iterations between host convergence checks.",
        {"src/backends/cuda/linear_system/linear_fused_pcg.cu"},
        Json{{"minimum", 1}});

    add("line_search/max_iter",
        IndexT{8},
        "integer",
        "Maximum backtracking iterations per Newton step.",
        {"src/backends/cuda/line_search"},
        Json{{"minimum", 1}});
    add("line_search/report_energy",
        IndexT{0},
        "integer",
        "Log the energy contribution of every line-search reporter.",
        {"src/backends/cuda/line_search"},
        flag);

    add("contact/enable",
        IndexT{1},
        "integer",
        "Enable contact detection and response systems.",
        {"src/backends/cuda/contact_system"},
        flag);
    add("contact/d_hat",
        Float{0.01},
        "number",
        "Absolute IPC activation distance.",
        {"src/backends/cuda/contact_system/global_contact_manager.cu"},
        Json{{"exclusiveMinimum", 0.0}},
        "m");
    add("contact/d_hat_relative",
        Float{0.0},
        "number",
        "Positive values replace d_hat with this fraction of the rest-scene diagonal.",
        {"src/backends/cuda/contact_system/global_contact_manager.cu"});
    add("contact/friction/enable",
        IndexT{1},
        "integer",
        "Enable frictional contact energy.",
        {"src/backends/cuda/contact_system"},
        flag);
    add("contact/eps_velocity",
        Float{0.01_m / 1.0_s},
        "number",
        "Absolute friction transition velocity.",
        {"src/backends/cuda/contact_system"},
        Json{{"exclusiveMinimum", 0.0}},
        "m/s");
    add("contact/eps_velocity_relative",
        Float{0.0},
        "number",
        "Positive values replace eps_velocity with this fraction of the rest-scene diagonal.",
        {"src/backends/cuda/contact_system/global_contact_manager.cu"});
    add("contact/constitution",
        std::string{"ipc"},
        "string",
        "Select the IPC or augmented-Lagrangian IPC pipeline.",
        {"src/backends/cuda/pipeline", "src/backends/cuda/engine"},
        Json{{"enum", Json::array({"ipc", "al-ipc"})}});

    add("contact/al-ipc/mu_scale_fem",
        Float{5e7},
        "number",
        "FEM augmented-Lagrangian penalty estimate scale.",
        {"src/backends/cuda/contact_system"},
        Json{{"exclusiveMinimum", 0.0}});
    add("contact/al-ipc/mu_scale_abd",
        Float{1e5},
        "number",
        "Affine-body augmented-Lagrangian penalty estimate scale.",
        {"src/backends/cuda/contact_system"},
        Json{{"exclusiveMinimum", 0.0}});
    add("contact/al-ipc/toi_threshold",
        Float{0.1},
        "number",
        "Time-of-impact threshold for AL active-set handling.",
        {"src/backends/cuda/contact_system"},
        Json{{"exclusiveMinimum", 0.0}, {"maximum", 1.0}});
    add("contact/al-ipc/alpha_lower_bound",
        Float{1e-6},
        "number",
        "Lower bound for an AL step length.",
        {"src/backends/cuda/line_search"},
        Json{{"exclusiveMinimum", 0.0}, {"maximum", 1.0}});
    add("contact/al-ipc/decay_factor",
        Float{0.3},
        "number",
        "AL penalty and constraint decay factor.",
        {"src/backends/cuda/contact_system"},
        Json{{"exclusiveMinimum", 0.0}, {"exclusiveMaximum", 1.0}});

    add("contact/adaptive/min_kappa",
        Float{100.0_MPa},
        "number",
        "Fallback lower bound for adaptive contact resistance.",
        {"src/backends/cuda/contact_system/global_contact_manager.cu"},
        Json{{"exclusiveMinimum", 0.0}},
        "Pa");
    add("contact/adaptive/init_kappa",
        Float{1.0_GPa},
        "number",
        "Initial adaptive contact resistance.",
        {"src/backends/cuda/contact_system"},
        Json{{"exclusiveMinimum", 0.0}},
        "Pa");
    add("contact/adaptive/max_kappa",
        Float{100.0_GPa},
        "number",
        "Fallback upper bound for adaptive contact resistance.",
        {"src/backends/cuda/contact_system/global_contact_manager.cu"},
        Json{{"exclusiveMinimum", 0.0}},
        "Pa");
    add("contact/adaptive/kappa_eval_scale",
        Float{1e-16},
        "number",
        "Evaluation scale for the scene-adaptive kappa corridor.",
        {"src/backends/cuda/contact_system/global_contact_manager.cu"},
        Json{{"exclusiveMinimum", 0.0}});

    add("collision_detection/method",
        std::string{"info_stackless_bvh"},
        "string",
        "Select the broad-phase trajectory filter.",
        {"src/backends/cuda/collision_detection/filters"},
        Json{{"enum", Json::array({"info_stackless_bvh", "info_stackless_bvh_v0", "stackless_bvh", "linear_bvh"})}});
    add("sanity_check/enable",
        IndexT{1},
        "integer",
        "Run pre-initialization intersection and distance checks.",
        {"src/core/core/internal/world.cpp"},
        flag);
    add("sanity_check/mode",
        std::string{"normal"},
        "string",
        "Select diagnostic-geometry output (normal) or a report-only check (quiet).",
        {"src/sanity_check", "src/backends/cuda/sanity_check"},
        Json{{"enum", Json::array({"normal", "quiet"})}});
    add("diff_sim/enable",
        IndexT{0},
        "integer",
        "Initialize differentiable-simulation state.",
        {"src/core/core/diff_sim.cpp", "src/backends/cuda"},
        flag);

    add("extras/debug/dump_surface",
        IndexT{0},
        "integer",
        "Dump intermediate surface state during the nonlinear solve.",
        {"src/backends/cuda"},
        flag);
    add("extras/debug/dump_linear_system",
        IndexT{0},
        "integer",
        "Dump assembled global linear systems.",
        {"src/backends/cuda/linear_system"},
        flag);
    add("extras/debug/dump_linear_pcg",
        IndexT{0},
        "integer",
        "Dump linear-PCG vectors.",
        {"src/backends/cuda/linear_system/linear_pcg.cu"},
        flag);
    add("extras/debug/dump_mas_matrices",
        IndexT{0},
        "integer",
        "Dump matrices assembled by the MAS FEM preconditioner.",
        {"src/backends/cuda/finite_element/fem_mas_preconditioner.cu"},
        flag);
    add("extras/strict_mode/enable",
        IndexT{0},
        "integer",
        "Convert nonlinear and line-search limit warnings into engine errors.",
        {"src/backends/cuda/engine"},
        flag);

    return contract;
}

geometry::AttributeCollection default_scene_config() noexcept
{
    return make_scene_config_contract().config;
}

Json scene_config_schema()
{
    auto  contract = make_scene_config_contract();
    auto& config   = contract.config;
    auto  entries  = std::move(contract.entries);
    auto  names    = config.names();

    UIPC_ASSERT_THROW(entries.size() == names.size(),
                      "Scene config metadata has {} entries but the default config has {}.",
                      entries.size(),
                      names.size());
    for(auto& name : names)
    {
        UIPC_ASSERT_THROW(entries.contains(name),
                          "Scene config key '{}' has no metadata entry.",
                          name);
        auto attribute = config.find(name);
        UIPC_ASSERT_THROW(attribute, "Scene config attribute '{}' is missing.", name);
    }

    return Json{{"schemaVersion", 1},
                {"strictUnknownKeys", true},
                {"validatedAt", Json::array({"Scene construction", "World::init(scene)"})},
                {"entries", std::move(entries)}};
}

static SizeT numeric_component_count(const Json& value)
{
    if(value.is_number())
        return 1;
    if(!value.is_array())
        return 0;
    SizeT count = 0;
    for(auto& child : value)
        count += numeric_component_count(child);
    return count;
}

static void validate_finite(std::string_view path, const Json& value)
{
    if(value.is_number_float())
    {
        UIPC_ASSERT_THROW(std::isfinite(value.get<double>()),
                          "Invalid scene config '{}': values must be finite.",
                          path);
    }
    else if(value.is_array())
    {
        for(auto& child : value)
            validate_finite(path, child);
    }
}

static void validate_config_value(std::string_view path, const Json& value, const Json& entry)
{
    const auto type         = entry["type"].get<std::string>();
    bool       type_matches = (type == "number" && value.is_number())
                        || (type == "integer" && value.is_number_integer())
                        || (type == "string" && value.is_string())
                        || (type == "array" && value.is_array());
    UIPC_ASSERT_THROW(type_matches,
                      "Invalid scene config '{}': expected {}, got {} ({}).",
                      path,
                      type,
                      value.type_name(),
                      value.dump());

    if(entry.value("finite", false))
        validate_finite(path, value);
    if(entry.contains("componentCount"))
    {
        const auto expected = entry["componentCount"].get<SizeT>();
        UIPC_ASSERT_THROW(numeric_component_count(value) == expected,
                          "Invalid scene config '{}': expected {} numeric components, got {}.",
                          path,
                          expected,
                          numeric_component_count(value));
    }
    if(entry.contains("const"))
    {
        UIPC_ASSERT_THROW(value == entry["const"],
                          "Invalid scene config '{}': this reserved key must remain {}, got {}.",
                          path,
                          entry["const"].dump(),
                          value.dump());
    }
    if(entry.contains("enum"))
    {
        bool found = false;
        for(auto& candidate : entry["enum"])
            found |= candidate == value;
        UIPC_ASSERT_THROW(found,
                          "Invalid scene config '{}': expected one of {}, got {}.",
                          path,
                          entry["enum"].dump(),
                          value.dump());
    }

    if(!value.is_number())
        return;
    const auto number = value.get<double>();
    if(entry.contains("minimum"))
        UIPC_ASSERT_THROW(number >= entry["minimum"].get<double>(),
                          "Invalid scene config '{}': {} is below minimum {}.",
                          path,
                          number,
                          entry["minimum"].get<double>());
    if(entry.contains("exclusiveMinimum"))
        UIPC_ASSERT_THROW(number > entry["exclusiveMinimum"].get<double>(),
                          "Invalid scene config '{}': {} must be greater than {}.",
                          path,
                          number,
                          entry["exclusiveMinimum"].get<double>());
    if(entry.contains("maximum"))
        UIPC_ASSERT_THROW(number <= entry["maximum"].get<double>(),
                          "Invalid scene config '{}': {} exceeds maximum {}.",
                          path,
                          number,
                          entry["maximum"].get<double>());
    if(entry.contains("exclusiveMaximum"))
        UIPC_ASSERT_THROW(number < entry["exclusiveMaximum"].get<double>(),
                          "Invalid scene config '{}': {} must be less than {}.",
                          path,
                          number,
                          entry["exclusiveMaximum"].get<double>());
}

void validate_scene_config(const geometry::AttributeCollection& config)
{
    const auto  schema  = scene_config_schema();
    const auto& entries = schema["entries"];

    for(auto& name : config.names())
    {
        UIPC_ASSERT_THROW(entries.contains(name),
                          "Unknown mutable scene config attribute '{}'. Custom metadata belongs "
                          "on geometry, not Scene::config().",
                          name);
    }

    for(auto&& [name, entry] : entries.items())
    {
        auto attribute = config.find(name);
        UIPC_ASSERT_THROW(attribute, "Required scene config attribute '{}' is missing.", name);
        const auto actual_storage_type = std::string{attribute->type_name()};
        const auto expected_storage_type = entry["storageType"].get<std::string>();
        UIPC_ASSERT_THROW(actual_storage_type == expected_storage_type,
                          "Invalid scene config '{}': storage type is '{}', expected '{}'.",
                          name,
                          actual_storage_type,
                          expected_storage_type);
        validate_config_value(name, attribute->to_json(0), entry);
    }

    auto number = [&](std::string_view name)
    { return config.find(name)->to_json(0).get<double>(); };

    UIPC_ASSERT_THROW(number("newton/min_iter") <= number("newton/max_iter"),
                      "Invalid scene config: newton/min_iter must not exceed "
                      "newton/max_iter.");
    const auto min_kappa  = number("contact/adaptive/min_kappa");
    const auto init_kappa = number("contact/adaptive/init_kappa");
    const auto max_kappa  = number("contact/adaptive/max_kappa");
    UIPC_ASSERT_THROW(min_kappa <= init_kappa && init_kappa <= max_kappa,
                      "Invalid scene config: require contact/adaptive/min_kappa <= "
                      "init_kappa <= max_kappa (got {}, {}, {}).",
                      min_kappa,
                      init_kappa,
                      max_kappa);
}

static Json& nested_json(Json& j, const std::string_view path)
{
    size_t pos     = 0;
    Json*  current = &j;
    while(true)
    {
        size_t next_pos = path.find('/', pos);
        auto   key      = path.substr(pos, next_pos - pos);

        if(next_pos == std::string_view::npos)
        {
            return (*current)[key];
        }

        auto& child = (*current)[key];
        if(!child.is_object())
        {
            child = Json::object();
        }
        current = &child;
        pos     = next_pos + 1;
    }
}

Json to_config_json(const geometry::AttributeCollection& config)
{
    Json j;
    auto names = config.names();

    for(auto& name : names)
    {
        auto attr = config.find(name);
        UIPC_ASSERT_THROW(attr != nullptr, "Attribute '{}' not found in config.", name);
        auto& sub_json = nested_json(j, name);
        sub_json       = attr->to_json(0);
    }
    return j;
}

static const Json* find_nested_json(const Json& j, const std::string_view path)
{
    size_t      pos     = 0;
    const Json* current = &j;
    while(true)
    {
        size_t next_pos = path.find('/', pos);
        auto   key      = path.substr(pos, next_pos - pos);

        if(!current->is_object())
        {
            return nullptr;
        }

        auto it = current->find(key);
        if(it == current->end())
        {
            return nullptr;
        }

        if(next_pos == std::string_view::npos)
        {
            return &(*it);
        }

        current = &(*it);
        pos     = next_pos + 1;
    }
}

static void throw_on_unknown_config_keys(const geometry::AttributeCollection& schema,
                                         const Json&        j,
                                         const std::string& prefix)
{
    if(!j.is_object())
        return;
    for(const auto& [key, value] : j.items())
    {
        std::string path = prefix.empty() ? key : fmt::format("{}/{}", prefix, key);
        if(schema.find(path) != nullptr)
            continue;  // registered leaf key
        if(value.is_object())
        {
            // intermediate node: acceptable only if some registered key lives below it
            bool is_prefix_of_registered = false;
            for(auto& name : schema.names())
            {
                if(name.size() > path.size() && name.starts_with(path)
                   && name[path.size()] == '/')
                {
                    is_prefix_of_registered = true;
                    break;
                }
            }
            UIPC_ASSERT_THROW(is_prefix_of_registered,
                              "Unknown scene config key \"{}\": nothing is registered under it "
                              "(typo, or a missing default registration in scene_default_config.cpp)",
                              path);
            throw_on_unknown_config_keys(schema, value, path);
            continue;
        }
        UIPC_ASSERT_THROW(false,
                          "Unknown scene config key \"{}\" (typo, or a missing default "
                          "registration in scene_default_config.cpp)",
                          path);
    }
}

void from_config_json(geometry::AttributeCollection& config, const Json& j)
{
    // Reject unknown keys up front: the forward loop below only visits keys that
    // are registered in the default schema, so unregistered keys set by the user
    // would otherwise be silently dropped (a real source of "parameter set but
    // never applied" bugs).
    throw_on_unknown_config_keys(config, j, "");

    auto names = config.names();
    for(auto& name : names)
    {
        auto attr = config.find(name);
        UIPC_ASSERT_THROW(attr != nullptr, "Attribute '{}' not found in config.", name);
        auto sub_json = find_nested_json(j, name);
        if(sub_json != nullptr)
        {
            // wrap it in an array to use from_json_array
            Json wrapper_array = Json::array();
            wrapper_array.push_back(*sub_json);
            attr->from_json_array(wrapper_array);
        }
    }
    validate_scene_config(config);
}
}  // namespace uipc::core
