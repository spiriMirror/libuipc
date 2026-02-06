#pragma once
#include "rbc_uipc_config.h"
#include <stdint.h>

// ==================== 核心对象创建/销毁 ====================

RBC_UIPC_API void* uipc_create_engine(char const* backend, char const* workspace, char const* config);
RBC_UIPC_API void uipc_destroy_engine(void* engine);

RBC_UIPC_API void* uipc_create_world(void* engine);
RBC_UIPC_API void  uipc_destroy_world(void* world);

RBC_UIPC_API void* uipc_create_scene(char const* config_json);
RBC_UIPC_API void  uipc_destroy_scene(void* scene);

// ==================== StableNeoHookean ====================

RBC_UIPC_API void* uipc_create_stable_neo_hookean();
RBC_UIPC_API void  uipc_destroy_stable_neo_hookean(void* snk);

// Apply StableNeoHookean to a SimplicialComplex
// moduli_type: 0=lame, 1=youngs_shear, 2=youngs_poisson
RBC_UIPC_API void uipc_stable_neo_hookean_apply_to(
    void* snk, void* sc, int moduli_type, double param1, double param2, double mass_density);

// ==================== AffineBodyConstitution ====================

RBC_UIPC_API void* uipc_create_affine_body_constitution(char const* config_json);
RBC_UIPC_API void uipc_destroy_affine_body_constitution(void* abd);

// Apply AffineBodyConstitution to a SimplicialComplex
RBC_UIPC_API void uipc_affine_body_constitution_apply_to(void*  sc,
                                                         void*  abd,
                                                         double kappa,
                                                         double mass_density);

// ==================== ConstitutionTabular ====================

RBC_UIPC_API void uipc_constitution_tabular_insert(void* scene, void* constitution);

// ==================== ContactTabular ====================

RBC_UIPC_API void uipc_contact_tabular_default_model(void*  scene,
                                                     double friction_rate,
                                                     double resistance,
                                                     bool   enable);

RBC_UIPC_API void* uipc_contact_tabular_default_element(void* scene);

RBC_UIPC_API void uipc_destroy_contact_element(void* element);