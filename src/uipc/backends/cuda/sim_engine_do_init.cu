#include <sim_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>
#include <sim_system_auto_register.h>
#include <log_pattern_guard.h>
#include <global_surface_manager.h>
#include <global_vertex_manager.h>
#include <dof_predictor.h>
#include <line_search/line_searcher.h>
#include <gradient_hessian_computer.h>
#include <linear_system/global_linear_system.h>

namespace uipc::backend::cuda
{
void SimEngine::build()
{
    // find those engine-aware topo systems
    m_global_vertex_manager     = find<GlobalVertexManager>();
    m_global_surface_manager    = find<GlobalSurfaceManager>();
    m_dof_predictor             = find<DoFPredictor>();
    m_line_searcher             = find<LineSearcher>();
    m_gradient_hessian_computer = find<GradientHessianComputer>();
    m_global_linear_system      = find<GlobalLinearSystem>();
}

void SimEngine::init_scene()
{
    auto& info        = m_world_visitor->scene().info();
    m_newton_tol      = info["newton"]["tolerance"];
    m_newton_max_iter = info["newton"]["max_iter"];
    Vector3 gravity   = info["gravity"];
    Float   dt        = info["dt"];
    m_abs_tol         = gravity.norm() * dt * dt / 2;
}

void SimEngine::do_init(backend::WorldVisitor v)
{
    LogGuard guard;

    m_world_visitor = make_unique<backend::WorldVisitor>(v);

    // 1) ConstitutionRegister all systems
    m_state = SimEngineState::RegisterSystems;
    register_all_systems();

    // 2) Build the relationships between systems
    m_state = SimEngineState::BuildSystems;
    for(auto&& [k, s] : m_system_collection.m_sim_systems)
        s->build();
    build();

    // 3) Trigger the init_scene event, systems register their actions will be called here
    m_state = SimEngineState::InitScene;
    {
        init_scene();
        event_init_scene();

        m_global_vertex_manager->init_vertex_info();
        m_global_surface_manager->init_surface_info();
    }

    // 4) Any creation and deletion of objects after this point will be pending
    auto scene_visitor = m_world_visitor->scene();
    scene_visitor.begin_pending();
}
}  // namespace uipc::backend::cuda