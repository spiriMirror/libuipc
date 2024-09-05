#include <sim_engine.h>
#include <uipc/common/log.h>
#include <log_pattern_guard.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_trajectory_filter.h>
#include <dof_predictor.h>
#include <line_search/line_searcher.h>
#include <gradient_hessian_computer.h>
#include <linear_system/global_linear_system.h>
#include <fstream>

namespace uipc::backend::cuda
{
void SimEngine::build()
{
    // 1) build all systems
    build_systems();

    // 2) find those engine-aware topo systems
    m_global_vertex_manager     = &require<GlobalVertexManager>();
    m_dof_predictor             = &require<DoFPredictor>();
    m_line_searcher             = &require<LineSearcher>();
    m_gradient_hessian_computer = &require<GradientHessianComputer>();
    m_global_linear_system      = &require<GlobalLinearSystem>();

    m_global_simplicial_surface_manager   = find<GlobalSimpicialSurfaceManager>();
    m_global_contact_manager   = find<GlobalContactManager>();
    m_global_trajectory_filter = find<GlobalTrajectoryFilter>();

    // 3) dump system info
    dump_system_info();
}

void SimEngine::init_scene()
{
    auto& info            = m_world_visitor->scene().info();
    m_newton_velocity_tol = info["newton"]["velocity_tol"];
    m_newton_max_iter     = info["newton"]["max_iter"];
    m_friction_enabled    = info["contact"]["friction"]["enable"];
    Vector3 gravity       = info["gravity"];
    Float   dt            = info["dt"];

    m_abs_tol = m_newton_velocity_tol * dt;

    // early init:
    [[maybe_unuse]] m_on_init_scene.view();
    [[maybe_unuse]] m_on_rebuild_scene.view();
    [[maybe_unuse]] m_on_write_scene.view();

    event_init_scene();

    // some systems need to be initialized after the scene is built
    m_global_vertex_manager->init_vertex_info();
    m_global_simplicial_surface_manager->init_surface_info();
    if(m_global_contact_manager)
        m_global_contact_manager->compute_d_hat();
}

void SimEngine::do_init(backend::WorldVisitor v)
{
    LogGuard guard;
    try
    {
        m_world_visitor = make_unique<backend::WorldVisitor>(v);

        // 1. Build all the systems and their dependencies
        m_state = SimEngineState::BuildSystems;
        build();

        // 2. Trigger the init_scene event, systems register their actions will be called here
        m_state = SimEngineState::InitScene;
        init_scene();

        // 3. Any creation and deletion of objects after this point will be pending
        auto scene_visitor = m_world_visitor->scene();
        scene_visitor.begin_pending();
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("SimEngine init error: {}", e.what());
    }
}
}  // namespace uipc::backend::cuda