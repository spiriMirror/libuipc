#include <sim_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>
#include <log_pattern_guard.h>
#include <global_geometry/global_surface_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_dcd_filter.h>
#include <collision_detection/global_ccd_filter.h>
#include <dof_predictor.h>
#include <line_search/line_searcher.h>
#include <gradient_hessian_computer.h>
#include <linear_system/global_linear_system.h>
#include <uipc/backends/module.h>
#include <fstream>

namespace uipc::backend::cuda
{
void SimEngine::build()
{
    // 1) build all systems
    build_systems();

    m_on_init_scene.init();
    m_on_rebuild_scene.init();
    m_on_write_scene.init();

    // 2) find those engine-aware topo systems
    m_global_vertex_manager     = &require<GlobalVertexManager>();
    m_dof_predictor             = &require<DoFPredictor>();
    m_line_searcher             = &require<LineSearcher>();
    m_gradient_hessian_computer = &require<GradientHessianComputer>();
    m_global_linear_system      = &require<GlobalLinearSystem>();

    m_global_surface_manager = find<GlobalSimpicialSurfaceManager>();
    m_global_contact_manager = find<GlobalContactManager>();
    m_global_dcd_filter      = find<GlobalDCDFilter>();
    m_global_ccd_filter      = find<GlobalCCDFilter>();

    // 3) dump system info
    dump_system_info();
}

void SimEngine::init_scene()
{
    auto& info        = m_world_visitor->scene().info();
    m_newton_tol      = info["newton"]["tolerance"];
    m_newton_max_iter = info["newton"]["max_iter"];
    Vector3 gravity   = info["gravity"];
    Float   dt        = info["dt"];

    m_abs_tol         = gravity.norm() * dt * dt;
    if(m_abs_tol == 0.0)
        m_abs_tol = std::numeric_limits<Float>::max();
}

void SimEngine::do_init(backend::WorldVisitor v)
{
    LogGuard guard;

    m_world_visitor = make_unique<backend::WorldVisitor>(v);

    // 1. Build all the systems and their dependencies
    m_state = SimEngineState::BuildSystems;
    build();

    // 2. Trigger the init_scene event, systems register their actions will be called here
    m_state = SimEngineState::InitScene;
    {
        init_scene();
        event_init_scene();

        m_global_vertex_manager->init_vertex_info();
        m_global_surface_manager->init_surface_info();
        if(m_global_contact_manager)
            m_global_contact_manager->compute_d_hat();
    }

    // 3. Any creation and deletion of objects after this point will be pending
    auto scene_visitor = m_world_visitor->scene();
    scene_visitor.begin_pending();
}
}  // namespace uipc::backend::cuda