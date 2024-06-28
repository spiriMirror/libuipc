#include <sim_engine.h>
#include <log_pattern_guard.h>
#include <dof_predictor.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_collision_detector.h>
#include <collision_detection/global_ccd_filter.h>
#include <line_search/line_searcher.h>
#include <gradient_hessian_computer.h>
#include <linear_system/global_linear_system.h>
#include <uipc/common/range.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    LogGuard guard;

    auto detect_candiates = [this](Float alpha)
    {
        if(m_global_collision_detector)
            m_global_collision_detector->detect_candidates(alpha);
    };

    auto compute_energy = [this](Float alpha) -> Float
    {
        // Step Forward => x = x_0 + alpha * dx
        m_line_searcher->step_forward(alpha);
        m_global_vertex_manager->step_forward(alpha);
        // Compute New Energy => E
        return m_line_searcher->compute_energy();
    };

    auto filter_toi = [this](Float alpha) -> Float
    {
        if(m_global_ccd_filter)
            return m_global_ccd_filter->filter_toi(alpha);
        return alpha;
    };

    // The Pipeline
    {
        // 1. Adaptive Parameter Calculation
        AABB vertex_bounding_box = m_global_vertex_manager->compute_vertex_bounding_box();
        detect_candiates(0.0);
        if(m_global_contact_manager)
            m_global_contact_manager->compute_adaptive_kappa();

        // 2. Predict Motion => x_tilde = x + v * dt
        m_state = SimEngineState::PredictMotion;
        m_dof_predictor->predict();

        // 3. Nonlinear-Newton Iteration
        Float box_size = vertex_bounding_box.diagonal().norm();
        Float tol      = std::min(m_newton_tol * box_size, m_abs_tol);

        for(auto&& iter : range(m_newton_max_iter))
        {
            // 1) Build Collision Pairs
            if(iter > 0)
                detect_candiates(0.0);

            m_state = SimEngineState::ComputeGradientHassian;
            // 2) Compute Contact Gradient and Hessian => G:Vector3, H:Matrix3x3
            if(m_global_contact_manager)
                m_global_contact_manager->compute_contact();

            // 3) Compute System Gradient and Hessian => G:Vector3, H:Matrix3x3
            // E.g. FEM/ABD ...
            m_gradient_hessian_computer->compute_gradient_hessian();


            m_state = SimEngineState::SolveGlobalLinearSystem;
            // 4) Solve Global Linear System => dx = A^-1 * b
            m_global_linear_system->solve();

            // 5) Get Max Movement => dx_max = max(|dx|), if dx_max < tol, break
            Float res = m_global_vertex_manager->compute_max_displacement();
            spdlog::info("Newton Iteration: {} Residual: {}/{}", iter, res, tol);
            if(res < tol)
                break;

            m_state = SimEngineState::LineSearch;
            // 6) Begin Line Search
            {
                Float alpha = 1.0;

                // update the candidates by the step length
                // so now we take in all the candidates that
                // can happen in the whole step
                detect_candiates(alpha);

                // Record Current State x to x_0
                m_line_searcher->record_start_point();
                m_global_vertex_manager->record_start_point();

                // Compute Current Energy => E_0
                Float E0 = m_line_searcher->compute_energy();

                // ccd filter
                alpha = filter_toi(alpha);

                SizeT max_line_search_iter = 1000;  // now just hard code it
                SizeT line_search_iter     = 0;
                while(line_search_iter++ < max_line_search_iter)  // Energy Test
                {
                    Float E = compute_energy(alpha);

                    bool energy_decrease = E < E0;  // Check Energy Decrease
                    bool no_inversion    = true;    // Check Inversion

                    bool success = energy_decrease && no_inversion;
                    if(success)
                        break;

                    // If not success, then shrink alpha
                    alpha /= 2;
                    E = compute_energy(alpha);
                }
            }
        }

        // 4. Update Velocity => v = (x - x_0) / dt
        m_state = SimEngineState::UpdateVelocity;
        m_dof_predictor->compute_velocity();
    }

    // Trigger the rebuild_scene event, systems register their actions will be called here
    m_state = SimEngineState::RebuildScene;
    {
        event_rebuild_scene();

        // TODO: rebuild the vertex and surface info
        // m_global_vertex_manager->rebuild_vertex_info();
        // m_global_surface_manager->rebuild_surface_info();
    }

    // After the rebuild_scene event, the pending creation or deletion can be solved
    auto scene = m_world_visitor->scene();
    scene.solve_pending();
}
}  // namespace uipc::backend::cuda
