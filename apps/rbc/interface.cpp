#include "interface.h"
#include <uipc/uipc.h>
#include <uipc/core/contact_element.h>
#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/constitution/affine_body_constitution.h>

using namespace uipc;
using namespace uipc::core;
using namespace uipc::geometry;
using namespace uipc::constitution;

// ==================== 核心对象创建/销毁 ====================

RBC_UIPC_API void* uipc_create_engine(char const* backend, char const* workspace, char const* config)
{
    Json config_json;
    if(config)
    {
        config_json = Json::parse(config);
    }
    else
    {
        config_json = Engine::default_config();
    }

    std::string_view ws = workspace ? workspace : "./";
    return new Engine{backend, ws, config_json};
}

RBC_UIPC_API void uipc_destroy_engine(void* engine)
{
    delete static_cast<Engine*>(engine);
}

RBC_UIPC_API void* uipc_create_world(void* engine)
{
    return new World{*static_cast<Engine*>(engine)};
}

RBC_UIPC_API void uipc_destroy_world(void* world)
{
    delete static_cast<World*>(world);
}

RBC_UIPC_API void* uipc_create_scene(char const* config_json)
{
    Json config;
    if(config_json)
    {
        config = Json::parse(config_json);
    }
    else
    {
        config = Scene::default_config();
    }
    return new Scene{config};
}

RBC_UIPC_API void uipc_destroy_scene(void* scene)
{
    delete static_cast<Scene*>(scene);
}

// ==================== StableNeoHookean ====================

RBC_UIPC_API void* uipc_create_stable_neo_hookean()
{
    return new StableNeoHookean{};
}

RBC_UIPC_API void uipc_destroy_stable_neo_hookean(void* snk)
{
    delete static_cast<StableNeoHookean*>(snk);
}

RBC_UIPC_API void uipc_stable_neo_hookean_apply_to(void* snk,
                                      void* sc,
                                      int moduli_type /*moduli_type 改为 enum*/,
                                      double param1,
                                      double param2,
                                      double mass_density)
{
    auto* snk_ptr = static_cast<StableNeoHookean*>(snk);
    auto* sc_ptr  = static_cast<SimplicialComplex*>(sc);

    ElasticModuli moduli = [&]
    {
        switch(moduli_type)
        {
            case 0:  // lame
                return ElasticModuli::lame(param1, param2);
                break;
            case 1:  // youngs_shear
                return ElasticModuli::youngs_shear(param1, param2);
                break;
            case 2:  // youngs_poisson
                return ElasticModuli::youngs_poisson(param1, param2);
                break;
            default:
                return ElasticModuli::youngs_poisson(120.0_kPa, 0.49);
                break;
        }
    }();

    snk_ptr->apply_to(*sc_ptr, moduli, mass_density);
}

// ==================== AffineBodyConstitution ====================

RBC_UIPC_API void* uipc_create_affine_body_constitution(char const* config_json)
{
    Json config;
    if(config)
    {
        config = Json::parse(config_json);
    }
    else
    {
        config = Engine::default_config();
    }
    return new AffineBodyConstitution{config};
}

RBC_UIPC_API void uipc_destroy_affine_body_constitution(void* abd)
{
    delete static_cast<AffineBodyConstitution*>(abd);
}

RBC_UIPC_API void uipc_affine_body_constitution_apply_to(void* sc, void* abd, double kappa, double mass_density)
{
    auto* abd_ptr = static_cast<AffineBodyConstitution*>(abd);
    auto* sc_ptr  = static_cast<SimplicialComplex*>(sc);

    abd_ptr->apply_to(*sc_ptr, kappa, mass_density);
}

// ==================== ConstitutionTabular ====================

RBC_UIPC_API void uipc_constitution_tabular_insert(void* scene, void* constitution)
{
    auto* scene_ptr = static_cast<Scene*>(scene);
    // Note: constitution should be a pointer to IConstitution-derived object
    // We need to cast it based on the actual type, but since both StableNeoHookean
    // and AffineBodyConstitution inherit from IConstitution, we can use a common approach
    // The pointer is actually to either StableNeoHookean or AffineBodyConstitution
    auto* constitution_ptr = static_cast<constitution::IConstitution*>(constitution);
    scene_ptr->constitution_tabular().insert(*constitution_ptr);
}

// ==================== ContactTabular ====================

RBC_UIPC_API void uipc_contact_tabular_default_model(void* scene, double friction_rate, double resistance, bool enable)
{
    auto* scene_ptr = static_cast<Scene*>(scene);
    scene_ptr->contact_tabular().default_model(friction_rate, resistance, enable);
}

RBC_UIPC_API void* uipc_contact_tabular_default_element(void* scene)
{
    auto* scene_ptr = static_cast<Scene*>(scene);
    // ContactElement is a small object, we need to allocate it on heap
    return new ContactElement{scene_ptr->contact_tabular().default_element()};
}

RBC_UIPC_API void uipc_destroy_contact_element(void* element)
{
    delete static_cast<ContactElement*>(element);
}
