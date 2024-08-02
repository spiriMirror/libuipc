#include <sim_engine.h>
#include <log_pattern_guard.h>
#include <dof_predictor.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_trajectory_filter.h>
#include <line_search/line_searcher.h>
#include <gradient_hessian_computer.h>
#include <linear_system/global_linear_system.h>
#include <uipc/common/range.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    auto pipeline = [this]
    {
        LogGuard guard;

        ++m_current_frame;

        spdlog::info(R"(>>> Begin Frame: {})", m_current_frame);

        Float alpha     = 1.0;
        Float ccd_alpha = 1.0;
        Float cfl_alpha = 1.0;

        auto detect_dcd_candidates = [this]
        {
            if(m_global_trajectory_filter)
            {
                m_global_trajectory_filter->detect(0.0);
                m_global_trajectory_filter->filter_active();
            }
        };

        auto detect_trajectory_candidates = [this](Float alpha)
        {
            if(m_global_trajectory_filter)
            {
                m_global_trajectory_filter->detect(alpha);
            }
        };

        auto filter_dcd_candidates = [this]
        {
            if(m_global_trajectory_filter)
            {
                m_global_trajectory_filter->filter_active();
            }
        };

        auto record_friction_candidates = [this]
        {
            if(m_global_trajectory_filter)
            {
                m_global_trajectory_filter->record_friction_candidates();
            }
        };

        auto compute_adaptive_kappa = [this]
        {
            if(m_global_contact_manager)
                m_global_contact_manager->compute_adaptive_kappa();
        };

        auto compute_contact = [this]
        {
            if(m_global_contact_manager)
                m_global_contact_manager->compute_contact();
        };

        auto cfl_condition = [&cfl_alpha, this](Float alpha)
        {
            //if(m_global_contact_manager)
            //{
            //    auto max_disp = m_global_vertex_manager->compute_max_displacement_norm();
            //    auto d_hat = m_global_contact_manager->d_hat();

            //    cfl_alpha = d_hat / max_disp;
            //    spdlog::info("CFL Condition: {} / {}, max dx: {}", cfl_alpha, alpha, max_disp);

            //    if(cfl_alpha < alpha)
            //    {
            //        return cfl_alpha;
            //    }
            //}
            return alpha;
        };

        auto filter_toi = [&ccd_alpha, this](Float alpha)
        {
            if(m_global_trajectory_filter)
            {
                ccd_alpha = m_global_trajectory_filter->filter_toi(alpha);
                if(ccd_alpha < alpha)
                {
                    spdlog::info("CCD Filter: {} < {}", ccd_alpha, alpha);
                    return ccd_alpha;
                }
            }
            // return ccd_alpha;
            return alpha;
        };

        auto compute_energy = [this, filter_dcd_candidates](Float alpha) -> Float
        {
            // Step Forward => x = x_0 + alpha * dx
            spdlog::info("Step Forward: {}", alpha);
            m_global_vertex_manager->step_forward(alpha);
            m_line_searcher->step_forward(alpha);

            // Update the collision pairs
            filter_dcd_candidates();

            // Compute New Energy => E
            return m_line_searcher->compute_energy(false);
        };

        /***************************************************************************************
        *                                  Core Pipeline
        ***************************************************************************************/

        {
            // 1. Adaptive Parameter Calculation
            AABB vertex_bounding_box =
                m_global_vertex_manager->compute_vertex_bounding_box();
            detect_dcd_candidates();
            compute_adaptive_kappa();

            // 2. Record Friction Candidates at the beginning of the frame
            record_friction_candidates();

            // 3. Predict Motion => x_tilde = x + v * dt
            m_state = SimEngineState::PredictMotion;
            m_dof_predictor->predict();

            // 4. Nonlinear-Newton Iteration
            Float box_size = vertex_bounding_box.diagonal().norm();
            Float tol      = m_newton_tol * box_size;
            Float res0     = 0.0;

            for(auto&& newton_iter : range(m_newton_max_iter))
            {
                // 1) Build Collision Pairs
                if(newton_iter > 0)
                    detect_dcd_candidates();

                // 2) Compute Contact Gradient and Hessian => G:Vector3, H:Matrix3x3
                m_state = SimEngineState::ComputeContact;
                compute_contact();

                // 3) Compute System Gradient and Hessian
                m_state = SimEngineState::ComputeGradientHessian;
                m_gradient_hessian_computer->compute_gradient_hessian();

                // 4) Solve Global Linear System => dx = A^-1 * b
                m_state = SimEngineState::SolveGlobalLinearSystem;
                m_global_linear_system->solve();

                // 5) Get Max Movement => dx_max = max(|dx|), if dx_max < tol, break
                m_global_vertex_manager->collect_vertex_displacements();
                Float res = m_global_vertex_manager->compute_axis_max_displacement();

                if(newton_iter == 0)
                    res0 = res;

                Float rel_tol = res == 0.0 ? 0.0 : res / res0;

                spdlog::info(">> Newton Iteration: {}. Residual/Tol/AbsTol/RelTol: {}/{}/{}/{}",
                             newton_iter,
                             res,
                             tol,
                             m_abs_tol,
                             rel_tol);

                // 6) Check Termination Condition
                // TODO: Maybe we can implement a class for termination condition in the future
                if(res <= tol && (res < m_abs_tol || res <= 1e-2 * res0))
                    break;

                // 7) Begin Line Search
                m_state = SimEngineState::LineSearch;
                {
                    alpha = 1.0;

                    // Record Current State x to x_0
                    m_line_searcher->record_start_point();
                    m_global_vertex_manager->record_start_point();
                    detect_trajectory_candidates(alpha);

                    // Compute Current Energy => E_0
                    Float E0 = m_line_searcher->compute_energy(true);  // initial energy

                    // CCD filter
                    alpha = filter_toi(alpha);

                    // CFL condition
                    alpha = cfl_condition(alpha);

                    // Compute Test Energy => E
                    Float E = compute_energy(alpha);

                    SizeT line_search_iter = 0;
                    while(line_search_iter++ < m_line_searcher->max_iter())  // Energy Test
                    {
                        bool energy_decrease = E <= E0;  // Check Energy Decrease

                        // TODO: Intersection & Inversion Check
                        bool no_inversion = true;

                        spdlog::info("Line Search Iteration: {} Alpha: {}, E/E0: {}, E0: {}",
                                     line_search_iter,
                                     alpha,
                                     E / E0,
                                     E0);

                        bool success = energy_decrease && no_inversion;
                        if(success)
                            break;

                        // If not success, then shrink alpha
                        alpha /= 2;
                        E = compute_energy(alpha);
                    }

                    if(line_search_iter >= m_line_searcher->max_iter())
                    {
                        spdlog::warn(
                            "Line Search Exits with Max Iteration: {} (Frame={}, Newton={})\n"
                            "E/E0: {}, E0:{}",
                            m_line_searcher->max_iter(),
                            m_current_frame,
                            newton_iter,
                            E / E0,
                            E0);
                    }
                }
            }

            // 5. Update Velocity => v = (x - x_0) / dt
            m_state = SimEngineState::UpdateVelocity;
            m_dof_predictor->compute_velocity();
            m_global_vertex_manager->record_prev_positions();
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

        spdlog::info("<<< End Frame: {}", m_current_frame);
    };

    try
    {
        pipeline();
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("Simulation Engine Exception: {}", e.what());
    }
}
}  // namespace uipc::backend::cuda
