#include <animator/global_animator.h>
#include <collision_detection/global_trajectory_filter.h>
#include <contact_system/global_contact_manager.h>
#include <diff_sim/global_diff_sim_manager.h>
#include <dof_predictor.h>
#include <fstream>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <gradient_hessian_computer.h>
#include <line_search/line_searcher.h>
#include <linear_system/global_linear_system.h>
#include <sim_engine.h>
#include <uipc/common/log.h>
#include <affine_body/affine_body_dynamics.h>
#include <finite_element/finite_element_method.h>
#include <global_geometry/global_body_manager.h>

namespace uipc::backend::cuda
{
void SimEngine::build()
{
    // 1) build all systems
    build_systems();

    // 2) find those engine-aware topo systems
    m_global_vertex_manager     = &require<GlobalVertexManager>();
    m_global_body_manager       = &require<GlobalBodyManager>();
    m_dof_predictor             = &require<DofPredictor>();
    m_line_searcher             = &require<LineSearcher>();
    m_gradient_hessian_computer = &require<GradientHessianComputer>();
    m_global_linear_system      = &require<GlobalLinearSystem>();

    m_global_simplicial_surface_manager = find<GlobalSimpicialSurfaceManager>();
    m_global_contact_manager            = find<GlobalContactManager>();
    m_global_trajectory_filter          = find<GlobalTrajectoryFilter>();
    m_global_animator                   = find<GlobalAnimator>();
    m_global_diff_sim_manager           = find<GlobalDiffSimManager>();

    m_affine_body_dynamics  = find<AffineBodyDynamics>();
    m_finite_element_method = find<FiniteElementMethod>();

    // 3) dump system info
    dump_system_info();
}

void SimEngine::init_scene()
{
    auto& info            = world().scene().info();
    m_newton_velocity_tol = info["newton"]["velocity_tol"];
    m_newton_max_iter     = info["newton"]["max_iter"];
    m_ccd_tol             = info["newton"]["ccd_tol"];
    m_friction_enabled    = info["contact"]["friction"]["enable"];
    m_strict_mode         = info["extras"]["strict_mode"]["enable"];
    Vector3 gravity       = info["gravity"];
    Float   dt            = info["dt"];

    m_abs_tol = m_newton_velocity_tol * dt;

    // 1. Before Common Scene Initialization
    if(m_affine_body_dynamics)
        m_affine_body_dynamics->init();
    if(m_finite_element_method)
        m_finite_element_method->init();
    m_global_body_manager->init();

    // 2. Common Scene Initialization Phase
    event_init_scene();

    // 3. After Common Scene Initialization
    // 3.1 Forwards
    m_global_vertex_manager->init();
    m_global_simplicial_surface_manager->init();

    if(m_global_contact_manager)
        m_global_contact_manager->init();
    if(m_global_animator)
        m_global_animator->init();
    m_global_linear_system->init();

    // 3.2 Backwards (if needed)
    if(m_global_diff_sim_manager)
        m_global_diff_sim_manager->init();
    //if(m_global_diff_contact_manager)
    //    m_global_diff_contact_manager->init();
    //if(m_abd_diff_sim_manager)
    //    m_abd_diff_sim_manager->init();
}

void SimEngine::do_init(InitInfo& info)
{
    try
    {
        // 1. Build all the systems and their dependencies
        m_state = SimEngineState::BuildSystems;
        build();

        // 2. Trigger the init_scene event, systems register their actions will be called here
        m_state = SimEngineState::InitScene;
        init_scene();

        // 3. Any creation and deletion of objects after this point will be pending
        world().scene().begin_pending();
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("SimEngine init error: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}
}  // namespace uipc::backend::cuda