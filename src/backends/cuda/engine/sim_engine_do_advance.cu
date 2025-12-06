#include <sim_engine.h>
#include <uipc/common/range.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_trajectory_filter.h>
#include <line_search/line_searcher.h>
#include <linear_system/global_linear_system.h>
#include <animator/global_animator.h>
#include <external_force/global_external_force_manager.h>
#include <diff_sim/global_diff_sim_manager.h>
#include <newton_tolerance/newton_tolerance_manager.h>
#include <time_integrator/time_integrator_manager.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    Float alpha     = 1.0;
    Float ccd_alpha = 1.0;
    Float cfl_alpha = 1.0;

    /***************************************************************************************
    *                                  Function Shortcuts
    ***************************************************************************************/

    auto detect_dcd_candidates = [this]
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Detect DCD Candidates"};
            m_global_trajectory_filter->detect(0.0);
            m_global_trajectory_filter->filter_active();
        }
    };

    auto detect_trajectory_candidates = [this](Float alpha)
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Detect Trajectory Candidates"};
            m_global_trajectory_filter->detect(alpha);
        }
    };

    auto filter_dcd_candidates = [this]
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Filter Contact Candidates"};
            m_global_trajectory_filter->filter_active();
        }
    };

    auto record_friction_candidates = [this]
    {
        if(m_global_trajectory_filter && m_friction_enabled)
        {
            m_global_trajectory_filter->record_friction_candidates();
        }
    };

    auto compute_adaptive_kappa = [this]
    {
        // TODO: now no effect
        if(m_global_contact_manager)
            m_global_contact_manager->compute_adaptive_kappa();
    };

    auto compute_dytopo_effect = [this]
    {
        // compute the dytopo effect gradient and hessian, containing:
        // 1) contact effect from contact pairs
        // 2) other dynamic topo effects, e.g. point picker, vertex stitch ...
        if(m_global_dytopo_effect_manager)
        {
            Timer timer{"Compute DyTopo Effect"};
            m_global_dytopo_effect_manager->compute_dytopo_effect();
        }
    };

    auto cfl_condition = [&cfl_alpha, this](Float alpha)
    {
        if(m_global_contact_manager)
        {
            cfl_alpha = m_global_contact_manager->compute_cfl_condition();
            if(cfl_alpha < alpha)
            {
                logger::info("CFL Filter: {} < {}", cfl_alpha, alpha);
                return cfl_alpha;
            }
        }

        return alpha;
    };

    auto filter_toi = [&ccd_alpha, this](Float alpha)
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Filter CCD TOI"};
            ccd_alpha = m_global_trajectory_filter->filter_toi(alpha);
            if(ccd_alpha < alpha)
            {
                logger::info("CCD Filter: {} < {}", ccd_alpha, alpha);
                return ccd_alpha;
            }
        }

        return alpha;
    };

    auto compute_energy = [this, filter_dcd_candidates](Float alpha) -> Float
    {
        // Step Forward => x = x_0 + alpha * dx
        m_global_vertex_manager->step_forward(alpha);
        m_line_searcher->step_forward(alpha);

        // Update the collision pairs
        filter_dcd_candidates();

        // Compute New Energy => E
        return m_line_searcher->compute_energy(false);
    };

    auto step_animation = [this]()
    {
        // NEW LIFECYCLE: Clear and prepare external forces BEFORE animator step
        if(m_global_external_force_manager)
        {
            Timer timer{"Clear External Forces"};
            m_global_external_force_manager->clear();
        }

        if(m_global_animator)
        {
            Timer timer{"Step Animation"};
            m_global_animator->step();
        }

        // NEW LIFECYCLE: Compute external force accelerations AFTER animator step
        if(m_global_external_force_manager)
        {
            Timer timer{"Compute External Force Accelerations"};
            m_global_external_force_manager->step();
        }
    };

    auto compute_animation_substep_ratio = [this](SizeT newton_iter)
    {
        // compute the ratio to the aim position.
        // dst = prev_position + ratio * (position - prev_position)
        if(m_global_animator)
        {
            m_global_animator->compute_substep_ratio(newton_iter);
            logger::info("Animation Substep Ratio: {}", m_global_animator->substep_ratio());
        }
    };

    auto animation_reach_target = [this]()
    {
        if(m_global_animator)
        {
            return m_global_animator->substep_ratio() >= 1.0;
        }
        return true;
    };

    auto convergence_check = [&](SizeT newton_iter) -> bool
    {
        if(m_dump_surface->view()[0])
        {
            dump_global_surface(fmt::format("dump_surface.{}.{}", m_current_frame, newton_iter));
        }

        NewtonToleranceManager::ResultInfo result_info;
        result_info.frame(m_current_frame);
        result_info.newton_iter(newton_iter);
        m_newton_tolerance_manager->check(result_info);

        if(!result_info.converged())
            return false;

        // ccd alpha should close to 1.0
        if(ccd_alpha < m_ccd_tol->view()[0])
            return false;

        if(!animation_reach_target())
            return false;

        return true;
    };

    auto update_diff_parm = [this]()
    {
        if(m_global_diff_sim_manager)
        {
            Timer timer{"Update Diff Parm"};
            m_global_diff_sim_manager->update();
        }
    };

    auto check_line_search_iter = [this](SizeT iter)
    {
        if(iter >= m_line_searcher->max_iter())
        {
            logger::warn("Line Search Exits with Max Iteration: {} (Frame={})",
                         m_line_searcher->max_iter(),
                         m_current_frame);

            if(m_strict_mode->view()[0])
            {
                throw SimEngineException("StrictMode: Line Search Exits with Max Iteration");
            }
        }
    };

    auto check_newton_iter = [this](SizeT iter)
    {
        if(iter >= m_newton_max_iter->view()[0])
        {
            logger::warn("Newton Iteration Exits with Max Iteration: {} (Frame={})",
                         m_newton_max_iter->view()[0],
                         m_current_frame);

            if(m_strict_mode->view()[0])
            {
                throw SimEngineException("StrictMode: Newton Iteration Exits with Max Iteration");
            }
        }
        else
        {
            logger::info("Newton Iteration Converged with Iteration Count: {}, Bound: [{}, {}]",
                         iter,
                         m_newton_min_iter->view()[0],
                         m_newton_max_iter->view()[0]);
        }
    };

    /***************************************************************************************
    *                                  Core Pipeline
    ***************************************************************************************/

    // Abort on exception if the runtime check is enabled for debugging
    constexpr bool AbortOnException = uipc::RUNTIME_CHECK;

    auto pipeline = [&]() noexcept(AbortOnException)
    {
        Timer timer{"Pipeline"};

        ++m_current_frame;

        logger::info(R"(>>> Begin Frame: {})", m_current_frame);

        // Rebuild Scene
        {
            Timer timer{"Rebuild Scene"};
            // Trigger the rebuild_scene event, systems register their actions will be called here
            m_state = SimEngineState::RebuildScene;
            {
                event_rebuild_scene();

                // TODO: rebuild the vertex and surface info
                // m_global_vertex_manager->rebuild_vertex_info();
                // m_global_surface_manager->rebuild_surface_info();
            }

            // After the rebuild_scene event, the pending creation or deletion can be solved
            world().scene().solve_pending();

            // Update the diff parms
            update_diff_parm();
        }

        // Simulation:
        {
            Timer timer{"Simulation"};

            // 1. Record Friction Candidates at the beginning of the frame
            record_friction_candidates();
            m_global_vertex_manager->update_attributes();
            m_global_vertex_manager->record_prev_positions();

            // 2. Adaptive Parameter Calculation
            detect_dcd_candidates();
            compute_adaptive_kappa();

            // 3. Predict Motion => x_tilde = x + v * dt
            m_state = SimEngineState::PredictMotion;
            // MUST step animation before predicting dof
            // some animation may provide information for DOF prediction
            step_animation();
            m_time_integrator_manager->predict_dof();


            // 4. Nonlinear-Newton Iteration
            m_newton_tolerance_manager->pre_newton(m_current_frame);

            auto   newton_max_iter = m_newton_max_iter->view()[0];
            auto   newton_min_iter = m_newton_min_iter->view()[0];
            IndexT newton_iter     = 0;
            for(; newton_iter < newton_max_iter; ++newton_iter)
            {
                Timer timer{"Newton Iteration"};

                // 1) Compute animation substep ratio
                compute_animation_substep_ratio(newton_iter);


                // 2) Build Collision Pairs
                if(newton_iter > 0)
                    detect_dcd_candidates();


                // 3) Compute Dynamic Topo Effect Gradient and Hessian => G:Vector3, H:Matrix3x3
                //    - Contact Effect
                //    - Other DyTopo Effects
                m_state = SimEngineState::ComputeDyTopoEffect;
                compute_dytopo_effect();


                // 4) Solve Global Linear System => dx = A^-1 * b
                m_state = SimEngineState::SolveGlobalLinearSystem;
                {
                    Timer timer{"Solve Global Linear System"};
                    m_global_linear_system->solve();
                }


                // 5) Collect Vertex Displacements Globally
                m_global_vertex_manager->collect_vertex_displacements();


                // 6) Check Termination Condition
                bool converged  = convergence_check(newton_iter);
                bool terminated = converged && (newton_iter >= newton_min_iter);
                if(terminated)
                    break;


                // 7) Begin Line Search
                m_state = SimEngineState::LineSearch;
                {
                    Timer timer{"Line Search"};

                    // Reset Alpha
                    alpha = 1.0;

                    // Record Current State x to x_0
                    m_line_searcher->record_start_point();
                    m_global_vertex_manager->record_start_point();
                    detect_trajectory_candidates(alpha);

                    // Compute Current Energy => E_0
                    Float E0 = m_line_searcher->compute_energy(true);  // initial energy

                    // CCD filter
                    alpha = filter_toi(alpha);

                    // CFL Condition
                    alpha = cfl_condition(alpha);

                    // * Step Forward => x = x_0 + alpha * dx
                    // Compute Test Energy => E
                    Float E = compute_energy(alpha);

                    if(!converged)
                    {
                        SizeT line_search_iter = 0;
                        while(line_search_iter < m_line_searcher->max_iter())
                        {
                            Timer timer{"Line Search Iteration"};

                            // Check Energy Decrease
                            // TODO: maybe better condition like Wolfe condition/Armijo condition in the future
                            bool energy_decrease = (E <= E0);

                            // Check Inversion
                            // TODO: Inversion check if needed
                            bool no_inversion = true;

                            bool success = energy_decrease && no_inversion;

                            if(success)
                                break;

                            // If not success, then shrink alpha
                            alpha /= 2;
                            E = compute_energy(alpha);

                            line_search_iter++;
                        }

                        // Check Line Search Iteration
                        // report warnings or throw exceptions if needed
                        check_line_search_iter(line_search_iter);
                    }
                }
            }

            // 5. Update Velocity => v = (x - x_0) / dt
            m_state = SimEngineState::UpdateVelocity;
            {
                Timer timer{"Update Velocity"};
                m_time_integrator_manager->update_state();
            }

            // Check Newton Iteration
            // report warnings or throw exceptions if needed
            check_newton_iter(newton_iter);
        }

        logger::info("<<< End Frame: {}", m_current_frame);
    };

    try
    {
        pipeline();
    }
    catch(const SimEngineException& e)
    {
        logger::error("Engine Advance Error: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}
}  // namespace uipc::backend::cuda
