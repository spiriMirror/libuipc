#include "scene_default_config.h"
#include <uipc/common/unit.h>

namespace uipc::core
{
geometry::AttributeCollection default_scene_config() noexcept
{
    geometry::AttributeCollection config;
    config.resize(1);
    config.create("dt", Float{0.01});
    config.create("gravity", Vector3{0.0, -9.8, 0.0});

    config.create("cfl/enable", IndexT{0});

    config.create("integrator/type", std::string{"bdf1"});

    config.create("newton/max_iter", IndexT{1024});
    config.create("newton/min_iter", IndexT{1});
    config.create("newton/use_adaptive_tol", IndexT{0});
    config.create("newton/velocity_tol", Float{0.05_m / 1.0_s});
    config.create("newton/ccd_tol", Float{1.0});
    config.create("newton/transrate_tol", Float{0.1 / 1.0_s});

    config.create("newton/semi_implicit/enable", IndexT{0});
    config.create("newton/semi_implicit/beta_tol", Float{1e-3});

    config.create("linear_system/tol_rate", Float{1e-3});
    config.create("linear_system/solver", std::string{"linear_pcg"});
    config.create("line_search/max_iter", IndexT{8});
    config.create("line_search/report_energy", IndexT{0});

    config.create("contact/enable", IndexT{1});
    config.create("contact/friction/enable", IndexT{1});
    config.create("contact/constitution", std::string{"ipc"});
    config.create("contact/d_hat", Float{0.01});
    config.create("contact/adaptive/min_kappa", Float{100.0_MPa});
    config.create("contact/adaptive/init_kappa", Float{1.0_GPa});
    config.create("contact/adaptive/max_kappa", Float{100.0_GPa});
    config.create("contact/eps_velocity", Float{0.01_m / 1.0_s});

    // default:
    //  - stackless_bvh
    // or:
    //  - linear_bvh (slower)
    config.create("collision_detection/method", std::string{"stackless_bvh"});

    config.create("sanity_check/enable", IndexT{1});
    config.create("sanity_check/mode", std::string{"normal"});

    config.create("diff_sim/enable", IndexT{0});

    config.create("extras/debug/dump_surface", IndexT{0});
    config.create("extras/debug/dump_linear_system", IndexT{0});
    config.create("extras/debug/dump_linear_pcg", IndexT{0});

    config.create("extras/strict_mode/enable", IndexT{0});

    return config;
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
        UIPC_ASSERT(attr != nullptr, "Attribute '{}' not found in config.", name);
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

void from_config_json(geometry::AttributeCollection& config, const Json& j)
{
    auto names = config.names();
    for(auto& name : names)
    {
        auto attr = config.find(name);
        UIPC_ASSERT(attr != nullptr, "Attribute '{}' not found in config.", name);
        auto sub_json = find_nested_json(j, name);
        if(sub_json != nullptr)
        {
            // wrap it in an array to use from_json_array
            Json wrapper_array = Json::array();
            wrapper_array.push_back(*sub_json);
            attr->from_json_array(wrapper_array);
        }
    }
}
}  // namespace uipc::core
