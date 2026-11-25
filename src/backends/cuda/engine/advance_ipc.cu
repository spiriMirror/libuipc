/*
 * NVIDIA contributions: line-search reuse integration for trajectory bounds
 * and active-contact count readback; reuse of trajectory candidates, the
 * frame-start active set and the last trial energy, trajectory BVH refit for
 * the whole Newton loop, the line-search prologue and first-trial energies
 * overlapped with detection and CCD, the prelaunched assembly hooks, the cubic
 * refinement of backtracked steps, the cached line-search energy graph with an
 * asynchronous initial energy, batched start-point copies, step forward and
 * post-solve retrieval, batched count read-backs and the prelaunched trial
 * contact energies; the elastic-producer prelaunch right after the contact
 * assembly launch, and the TOI fork from the detection hook before the
 * candidate-count read-back.
 *
 * The NVIDIA notice applies to the contributions described above, not to the
 * pre-existing libuipc or other contributors' code.
 */

/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sim_engine.h>
#include <uipc/common/range.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <contact_system/global_contact_manager.h>
#include <collision_detection/global_trajectory_filter.h>
#include <line_search/line_searcher.h>
#include <line_search/cubic_line_minimizer.h>
#include <linear_system/global_linear_system.h>
#include <animator/global_animator.h>
#include <diff_sim/global_diff_sim_manager.h>
#include <newton_tolerance/newton_tolerance_manager.h>
#include <time_integrator/time_integrator_manager.h>
#include <utility>

namespace uipc::backend::cuda
{
void SimEngine::advance()
{
    Float alpha     = 1.0;
    Float beta      = 1.0;
    Float ccd_alpha = 1.0;
    Float cfl_alpha = 1.0;

    /***************************************************************************************
    *                                  Function Shortcuts
    ***************************************************************************************/

    auto detect_dcd_candidates = [this](bool reuse_bvh_topology)
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Detect DCD Candidates"};
            m_global_trajectory_filter->detect(0.0, reuse_bvh_topology);
            m_global_trajectory_filter->filter_active();
        }
    };

    // The previous frame's last line-search trial filtered its active set at
    // the positions it left behind. While they are unchanged at the
    // frame start, that active set is the one the DCD pass would build.
    auto reuse_active_set = [this]
    {
        return m_global_trajectory_filter
               && m_global_trajectory_filter->reuse_active_set();
    };

    auto record_active_set = [this]
    {
        if(m_global_trajectory_filter)
            m_global_trajectory_filter->record_active_set();
    };

    // before_read_back: see GlobalTrajectoryFilter::DetectInfo
    auto detect_trajectory_candidates =
        [this](Float                        alpha,
               bool                         reuse_bvh_topology,
               const std::function<void()>& before_read_back = {})
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Detect Trajectory Candidates"};
            m_global_trajectory_filter->detect(alpha, reuse_bvh_topology, true, before_read_back);
        }
    };

    // The trajectory candidates of an earlier iteration of this frame serve
    // a step over which every vertex stays within axis_motion per axis of the
    // swept segment they were detected for.
    auto reuse_trajectory_candidates = [this](Float axis_motion)
    {
        return m_global_trajectory_filter
               && m_global_trajectory_filter->reuse_candidates(axis_motion);
    };

    // prelaunch: a trial energy follows (or may follow, then discarded), and
    // its contact energies are launched before the read-back of the active
    // set's counts (LineSearcher::prelaunch_energy).
    auto filter_dcd_candidates = [this](bool prelaunch = false)
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Filter Contact Candidates"};
            if(prelaunch)
                m_global_trajectory_filter->filter_active(
                    [this](cudaEvent_t filtered)
                    { m_line_searcher->prelaunch_energy(filtered); });
            else
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
        if(m_global_contact_manager)
            m_global_contact_manager->compute_adaptive_parameters();
    };

    auto compute_dytopo_effect = [this]
    {
        // compute the dytopo effect gradient and hessian, containing:
        // 1) contact effect from contact pairs
        // 2) other dynamic topo effects, e.g. point picker, vertex stitch ...
        // The contact-independent assembly producers are issued once the
        // DyTopo assembly kernels are enqueued: those lead the critical path,
        // and the producers run beside them and the contact distribution on
        // the side streams (ABD's were forked by prepare_prelaunch(); FEM's
        // light producers start from a point marked before the DyTopo work).
        if(m_global_dytopo_effect_manager)
        {
            Timer timer{"Compute DyTopo Effect"};
            GlobalDyTopoEffectManager::ComputeDyTopoEffectInfo info;
            info.after_leading_launch(
                [this] { m_global_linear_system->prelaunch_ahead(); });
            info.after_assembly_launch(
                [this] { m_global_linear_system->prelaunch_assembly(true); });
            m_global_dytopo_effect_manager->compute_dytopo_effect(info);
        }
        else
        {
            m_global_linear_system->prelaunch_assembly(false);
        }
    };

    auto cfl_condition = [&cfl_alpha, this](Float alpha)
    {
        if(m_global_contact_manager)
        {
            Timer timer{"Compute CFL Condition"};
            cfl_alpha = m_global_contact_manager->compute_cfl_condition();
            m_frame_last_cfl_alpha = cfl_alpha;
            if(cfl_alpha < alpha)
            {
                logger::info("CFL Filter: {} < {}", cfl_alpha, alpha);
                return cfl_alpha;
            }
        }

        return alpha;
    };

    // launched: the TOI was launched on the prologue stream (see below)
    auto filter_toi = [&ccd_alpha, this](Float alpha, bool launched = false)
    {
        if(m_global_trajectory_filter)
        {
            Timer timer{"Filter CCD TOI"};
            ccd_alpha = launched ? m_global_trajectory_filter->finish_filter_toi() :
                                   m_global_trajectory_filter->filter_toi(alpha);
            m_frame_last_ccd_toi = ccd_alpha;
            // ccd_alpha is a fraction of the swept step `alpha`; compose to
            // get the absolute step fraction
            Float absolute_alpha = alpha * ccd_alpha;
            if(absolute_alpha < alpha)
            {
                logger::info("CCD Filter: {} < {}", absolute_alpha, alpha);
                return absolute_alpha;
            }
        }

        return alpha;
    };

    // Stiff-GIPC design: cap the step with the feasible step over the active
    // contact set (each active pair keeps at least 20% of its current gap),
    // so the swept-AABB trajectory candidates generated afterwards are fewer.
    // launched: the step was launched on the prologue stream (see below)
    auto feasible_step = [this](Float alpha, bool launched = false)
    {
        if(m_global_contact_manager)
        {
            Timer timer{"Compute Feasible Step"};
            Float feasible_alpha =
                launched ? m_global_contact_manager->finish_feasible_step() :
                           m_global_contact_manager->compute_feasible_step();
            if(feasible_alpha < alpha)
            {
                logger::info("Feasible Step Filter: {} < {}", feasible_alpha, alpha);
                return feasible_alpha;
            }
        }

        return alpha;
    };

    // Line-search prologue overlap: the feasible step and the Newton
    // tolerance checks run on the line searcher's prologue stream beside the
    // candidate reuse check, and the CCD TOI beside E_0 and the first trial
    // step, instead of in front of them. Every result stays as in the serial
    // order: the trajectory detection waits for the feasible step, the first
    // trial is stepped again when the TOI or the CFL condition truncates it
    // (its contact pairs are filtered after the TOI), and the TOI reads the
    // recorded start positions, which the trial step does not touch. Off
    // with surface dumps (a redone trial would dump twice) and without
    // line-search trials.
    auto overlap_prologue = [this]
    {
        return m_global_trajectory_filter && !m_dump_surface->view()[0]
               && m_line_searcher->max_iter() > 0;
    };

    auto launch_prologue_checks = [this](cudaStream_t stream)
    {
        if(m_global_contact_manager)
            m_global_contact_manager->launch_feasible_step(stream);
        m_newton_tolerance_manager->launch_check(stream);
    };

    auto step_forward = [this](Float alpha)
    {
        // Step Forward => x = x_0 + alpha * dx: the vertex positions and the
        // reporters' DOFs in one kernel
        StepBatch steps{alpha};
        m_global_vertex_manager->step_forward(steps);
        m_line_searcher->step_forward(steps);
        steps.launch();
    };

    // launch: the trial's energy is evaluated next. The energy that reads
    // only the stepped DOFs is then launched before the trial's contact
    // pairs are filtered, so it runs beside filter_active.
    auto step_to_trial = [this, step_forward, filter_dcd_candidates](Float alpha, bool launch)
    {
        step_forward(alpha);
        if(launch)
            m_line_searcher->launch_energy();

        // Dumps surface after step forward
        if(m_dump_surface->view()[0])
        {
            dump_global_surface();
        }

        // Update the collision pairs
        filter_dcd_candidates(launch);
    };

    auto substep_ratio = [this]() -> Float {
        return m_global_animator ? m_global_animator->substep_ratio() : Float{1.0};
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
        if(!animation_reach_target())
            return false;

        if(m_semi_implicit_enabled)
        {
            // Ref: https://arxiv.org/abs/2512.12151, Algorithm 1.
            // newton/semi_implicit/K_min plays Stiff-GIPC's Kmin role: beta
            // accumulation only starts from this iteration on; it is not a
            // floor on the Newton iteration count (that is newton/min_iter,
            // default 0 = no forced minimum).
            auto k_min = m_semi_implicit_kmin->view()[0];
            auto eps   = m_semi_implicit_beta_tol;
            if(newton_iter >= k_min)
                beta = (1.0 - alpha) * beta;

            if(beta <= eps)
                return true;  // early terminate
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

    auto check_line_search_iter = [&](SizeT line_search_iter_after_loop, Float E0, Float last_E)
    {
        if(line_search_iter_after_loop >= m_line_searcher->max_iter())
        {
            m_frame_hit_line_search_limit = true;
            // alpha was halved once more after the last failed try,
            // so the last actually tried step length is 2*alpha
            Float alpha_last = 2.0 * alpha;
            Float abs_E0     = std::abs(E0);
            Float rel_E_increase = (last_E - E0) / (abs_E0 > 1e-30 ? abs_E0 : 1e-30);
            auto detail = fmt::format(
                "Line Search Exits with Max Iteration: {} (Frame={}, Newton={}, "
                "alpha_last={}, E0={:.6e}, E_last={:.6e}, rel_E_increase={:.3e}, "
                "ccd_alpha={}, cfl_alpha={})",
                m_line_searcher->max_iter(),
                m_current_frame,
                m_newton_iter,
                alpha_last,
                E0,
                last_E,
                rel_E_increase,
                ccd_alpha,
                cfl_alpha);
            logger::warn("{}", detail);

            if(m_strict_mode->view()[0])
            {
                throw SimEngineException("StrictMode: " + detail);
            }
        }
    };

    auto check_newton_iter = [this](IndexT newton_iter_after_loop)
    {
        auto newton_max = m_newton_max_iter->view()[0];
        if(newton_iter_after_loop >= newton_max)
        {
            m_frame_hit_newton_limit = true;
            logger::warn("Newton Iteration Exits with Max Iteration: {} (Frame={})",
                         newton_max,
                         m_current_frame);

            if(m_strict_mode->view()[0])
            {
                throw SimEngineException("StrictMode: Newton Iteration Exits with Max Iteration");
            }
        }
        else
        {
            logger::info("Newton Iteration Converged with Iteration Count: {}, Max: {}",
                         newton_iter_after_loop,
                         newton_max);
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
        reset_frame_stats();

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

            // 0. Process External Changes
            m_global_vertex_manager->update_attributes();
            [[maybe_unused]] AABB bbox =
                m_global_vertex_manager->compute_vertex_bounding_box();

            // 1. Record Friction Candidates at the beginning of the frame
            m_global_vertex_manager->record_prev_positions();
            record_friction_candidates();

            // 2. Predict Motion => x_tilde = x + v * dt
            m_state = SimEngineState::PredictMotion;
            // MUST step animation before predicting dof
            // some animation may provide information for DOF prediction
            step_animation_and_external_forces();
            m_time_integrator_manager->predict_dof();

            // 3. Adaptive Parameter Calculation
            bool reused_active_set    = reuse_active_set();
            m_frame_active_set_reused = reused_active_set;
            if(!reused_active_set)
                detect_dcd_candidates(false);
            compute_adaptive_kappa();

            // 4. Nonlinear-Newton Iteration
            m_newton_tolerance_manager->pre_newton(m_current_frame);

            auto newton_max_iter = m_newton_max_iter->view()[0];
            auto newton_min_iter = m_newton_min_iter->view()[0];
            beta                 = 1.0;
            IndexT newton_iter   = 0;

            // Energy of the last line-search trial, valid until the frame
            // ends. The next Newton iteration starts at that trial's position:
            // nothing in between moves a vertex, runs filter_active or changes
            // kappa or the friction set, so the trial's energy is that
            // iteration's E_0 unless the animation substep ratio moved.
            // (Energy reporting keeps one log entry per evaluation.)
            bool  has_trial_energy           = false;
            Float trial_energy               = 0.0;
            Float trial_energy_substep_ratio = 0.0;

            // Per-axis bound of the distance from the iteration's start point
            // to the swept segment of the last trajectory detection: zero after
            // a detecting iteration (the accepted position lies on that
            // segment), growing by the accepted step of every reusing one.
            Float candidate_drift = 0.0;
            for(; newton_iter < newton_max_iter; ++newton_iter)
            {
                Timer timer{"Newton Iteration"};
                m_newton_iter             = newton_iter;
                m_frame_newton_iterations = newton_iter + 1;

                // 1) Compute animation substep ratio
                compute_animation_substep_ratio(newton_iter);


                // 2) Collision pairs: iteration 0 uses the active set of the frame
                //    start (DCD pass or reused). Later iterations reuse the previous
                //    trajectory candidates: they cover the swept segment
                //    x_k -> x_k + alpha*dx_k, which contains the accepted position,
                //    and the last line-search trial already ran filter_active there.


                // 3) Compute Dynamic Topo Effect Gradient and Hessian => G:Vector3, H:Matrix3x3
                //    - Contact Effect
                //    - Other DyTopo Effects
                //    Contact-independent assembly producers run alongside.
                //    The FEM elastic producer may go right after the contact
                //    assembly kernels, beside them (prelaunch_ahead()).
                m_global_linear_system->prepare_prelaunch(
                    m_global_dytopo_effect_manager ?
                        m_global_dytopo_effect_manager->assembly_blocks() :
                        0);
                m_state = SimEngineState::ComputeDyTopoEffect;
                compute_dytopo_effect();


                // 4) Solve Global Linear System => dx = A^-1 * b
                //    (the solver's iteration count is collected after the line
                //    search, whose read-backs have waited for the solve)
                m_state = SimEngineState::SolveGlobalLinearSystem;
                {
                    Timer timer{"Solve Global Linear System"};
                    // the ABD and FEM steps are retrieved by the
                    // displacement collection below (one kernel each)
                    m_global_linear_system->solve(true);
                }


                // 5) Collect Vertex Displacements Globally (their copies
                //    run with the line search's start-point copies below,
                //    as one kernel)
                cuda_tool::CopyBatch start_copies;
                m_global_vertex_manager->collect_vertex_displacements(start_copies);

                // 7) Begin Line Search
                m_state = SimEngineState::LineSearch;
                {
                    Timer timer{"Line Search"};

                    // Reset Alpha
                    alpha = 1.0;

                    // Record Current State x to x_0
                    m_line_searcher->record_start_point(start_copies);
                    m_global_vertex_manager->record_start_point(start_copies);
                    start_copies.launch();

                    if(m_dump_surface->view()[0])
                    {
                        dump_global_surface_pre_ccd(newton_iter);
                    }

                    bool overlap = overlap_prologue();
                    if(overlap)
                        launch_prologue_checks(
                            m_line_searcher->m_prologue_stream.fork());

                    // Iterations k > 0 keep the trajectory candidates while the
                    // drift plus this step (at most max |dx| per axis, alpha <= 1)
                    // stays inside the detection skin; the max |dx| readback is
                    // shared with the serial velocity tolerance check.
                    Float axis_max_disp     = 0.0;
                    bool  reused_candidates = false;
                    if(newton_iter > 0 && m_global_trajectory_filter)
                    {
                        axis_max_disp =
                            m_global_vertex_manager->compute_axis_max_displacement();
                        reused_candidates =
                            reuse_trajectory_candidates(candidate_drift + axis_max_disp);
                    }

                    // The trajectory BVH keeps the topology of the frame start
                    // and is refitted: iteration 0 requests the rebuild when
                    // the frame start skipped the DCD pass (see the filter).
                    bool rebuild_bvh = newton_iter == 0 && reused_active_set;

                    // Stiff-GIPC design: cap the step with the feasible step
                    // over the active contact set first, then generate the
                    // swept-AABB trajectory candidates over the capped step
                    // (fewer candidates than over the full step). Reused
                    // candidates cover any step alpha <= 1, so the feasible
                    // step does not matter to them.
                    // Once the detection's candidate counts are read back,
                    // the TOI's stream is forked after the detection's work
                    // (again after a detection that repeats its read-back).
                    cudaStream_t toi_stream = nullptr;
                    auto         fork_toi   = [&]
                    {
                        if(overlap)
                            toi_stream = m_line_searcher->m_prologue_stream.fork();
                    };
                    alpha = feasible_step(alpha, overlap);
                    if(reused_candidates)
                    {
                        ++m_frame_trajectory_reuses;
                    }
                    else
                    {
                        detect_trajectory_candidates(alpha, !rebuild_bvh, fork_toi);
                        candidate_drift = 0.0;
                    }

                    // The TOI runs beside E_0 and the first trial step.
                    if(overlap)
                        m_global_trajectory_filter->launch_filter_toi(
                            alpha,
                            toi_stream ? toi_stream :
                                         m_line_searcher->m_prologue_stream.fork());

                    // Compute Current Energy => E_0. Evaluated, it is only
                    // launched here: E_0 is first compared after the first
                    // trial's energy, whose read-back waits for it in stream
                    // order, so the host steps the trial meanwhile.
                    Float E0         = 0.0;
                    bool  E0_pending = false;
                    if(has_trial_energy && trial_energy_substep_ratio == substep_ratio()
                       && !m_line_searcher->m_report_energy)
                    {
                        E0 = trial_energy;
                    }
                    else if(!m_line_searcher->m_report_energy)
                    {
                        m_line_searcher->launch_initial_energy();
                        E0_pending = true;
                    }
                    else
                    {
                        E0 = m_line_searcher->compute_energy(true);  // initial energy
                    }
                    auto initial_energy = [&]() -> Float
                    {
                        if(std::exchange(E0_pending, false))
                            E0 = m_line_searcher->initial_energy();
                        return E0;
                    };

                    // Overlapped, the first trial steps forward while the TOI
                    // runs, and the energy producers that read only the stepped
                    // DOFs are launched beside it. Its contact pairs are
                    // filtered only once the TOI and the CFL condition keep
                    // the step: a step beyond the TOI may penetrate, which
                    // filter_active asserts against.
                    bool  stepped     = overlap;
                    Float trial_alpha = alpha;
                    if(stepped)
                    {
                        step_forward(alpha);
                        m_line_searcher->launch_energy();
                    }

                    // CCD filter
                    alpha = filter_toi(alpha, overlap);

                    // CFL Condition (Stiff-GIPC design: the per-step speed cap
                    // applies only when some trajectory pair actually hits within
                    // this step; in free flight it must not bite)
                    if(ccd_alpha < 1.0)
                        alpha = cfl_condition(alpha);
                    stepped = stepped && alpha == trial_alpha;
                    if(stepped)
                        filter_dcd_candidates(true);
                    else
                        m_line_searcher->discard_energy();

                    // Line Search Iteration
                    bool converged = convergence_check(newton_iter);
                    bool terminated = converged && (newton_iter >= newton_min_iter);
                    SizeT line_search_iter = 0;
                    Float last_E = 0.0;  // E_0 unless a trial is evaluated
                    bool  last_E_evaluated = false;
                    // last rejected trial (alpha, E), for the opt-in refinement
                    Float rejected_alpha = 0.0;
                    Float rejected_E     = 0.0;
                    for(; line_search_iter < m_line_searcher->max_iter(); ++line_search_iter)
                    {
                        Timer timer{"Line Search Iteration"};
                        m_line_search_iter = line_search_iter;
                        ++m_frame_line_search_trials;
                        m_frame_last_line_search_alpha = alpha;

                        // Compute Test Energy:
                        //  * Step Forward => x = x_0 + alpha * dx
                        //  * Compute New Energy => E
                        if(!std::exchange(stepped, false))
                            step_to_trial(alpha, true);

                        Float E = m_line_searcher->compute_energy(false);
                        last_E  = E;
                        last_E_evaluated = true;

                        // To prevent numerical energy (fake-) increasing caused by tiny dx
                        if(converged)
                            break;

                        // Check Energy Decrease
                        // TODO: maybe better condition like Wolfe condition/Armijo condition in the future
                        bool energy_decrease = (E <= initial_energy());

                        // Check Inversion
                        // TODO: Inversion check if needed
                        bool no_inversion = true;

                        bool success = energy_decrease && no_inversion;
                        if(success)
                        {
                            // line_search/cubic_refinement (off by default):
                            // a backtracked step is tried once more at the
                            // minimizer of the cubic through E0, the slope
                            // g.dx and the rejected and accepted trials; the
                            // lower of the two accepted energies wins.
                            if(m_line_searcher->cubic_refinement() && rejected_alpha > 0.0
                               && line_search_iter + 2 < m_line_searcher->max_iter())
                            {
                                Float slope = m_global_linear_system->newton_step_slope();
                                Float refined_alpha = cubic_line_minimizer(
                                    initial_energy(), slope, alpha, E, rejected_alpha, rejected_E);
                                if(refined_alpha > 0.05 * rejected_alpha
                                   && refined_alpha < rejected_alpha
                                   && std::abs(refined_alpha - alpha) > 0.25 * alpha)
                                {
                                    ++m_frame_line_search_trials;
                                    step_to_trial(refined_alpha, true);
                                    Float refined_E =
                                        m_line_searcher->compute_energy(false);
                                    if(refined_E <= E)
                                    {
                                        alpha  = refined_alpha;
                                        last_E = refined_E;
                                    }
                                    else
                                    {
                                        // Restore the accepted trial and its contact energies.
                                        step_to_trial(alpha, false);
                                        last_E = m_line_searcher->compute_energy(false);
                                    }
                                    m_frame_last_line_search_alpha = alpha;
                                }
                            }
                            break;
                        }

                        // If not success, then shrink alpha
                        rejected_alpha = alpha;
                        rejected_E     = E;
                        alpha /= 2;
                    }

                    m_frame_linear_solver_iterations +=
                        m_global_linear_system->last_solve_iterations();

                    if(!last_E_evaluated)
                        last_E = initial_energy();

                    // Check Line Search Iteration: report warnings or throw exceptions if needed
                    check_line_search_iter(line_search_iter, initial_energy(), last_E);

                    // The last trial moved every vertex by at most
                    // alpha * max |dx| per axis.
                    if(reused_candidates)
                        candidate_drift += m_frame_last_line_search_alpha * axis_max_disp;

                    has_trial_energy           = true;
                    trial_energy               = last_E;
                    trial_energy_substep_ratio = substep_ratio();

                    // newton/min_iter is a pure hard floor (default 0 = off);
                    // the semi-implicit Kmin lives in newton/semi_implicit/K_min
                    if(terminated)
                    {
                        m_frame_converged = true;
                        break;
                    }
                }
            }

            // The positions are those of the last filter_active.
            record_active_set();

            // 5. Update Velocity => v = (x - x_0) / dt
            m_state = SimEngineState::UpdateVelocity;
            {
                Timer timer{"Update Velocity"};
                m_time_integrator_manager->update_state();
            }

            // Check Newton Iteration
            // report warnings or throw exceptions if needed
            check_newton_iter(newton_iter);
            m_frame_completed = true;
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
    catch(const std::exception& e)
    {
        UIPC_ASSERT(false, "Unexpected Exception: {}", e.what());
    }
}
}  // namespace uipc::backend::cuda
