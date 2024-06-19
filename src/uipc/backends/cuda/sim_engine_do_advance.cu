#include <sim_engine.h>
#include <log_pattern_guard.h>

namespace uipc::backend::cuda
{
void SimEngine::do_advance()
{
    LogGuard guard;

    // 1. Trigger the rebuild_scene event, systems register their actions will be called here
    m_state = SimEngineState::RebuildScene;
    event_rebuild_scene();

    // 2. After the rebuild_scene event, the pending creation or deletion can be solved
    auto scene = m_world_visitor->scene();
    scene.solve_pending();

    // 3. Begin the pipeline
    {
        // a. Predict Motion => x_tilde = x + v * dt

        // b. Build Collision Pairs

        // c. Select Adaptive Kappa

        // d. Begin Newton Iteration
        Float tol  = 1e-6;     // now just hard code it
        Float res  = 2 * tol;  // now just hard code it
        SizeT iter = 1000;     // now just hard code it
        while(iter--)
        {
            // 1) Build Collision Pairs

            // 2) Compute Contact Gradient and Hessian => G:Vector3, H:Matrix3x3

            // 3) Compute System Gradient and Hessian => G:Vector3, H:Matrix3x3
            // E.g. FEM/ABD ...

            // 4) Assemble Global Linear System => A:SparseMatrix of H, b:DenseVector of G

            // 5) Solve Global Linear System => dx = A^-1 * b

            // 6) Get Max Movement => dx_max = max(|dx|), if dx_max < tol, break
            res = 0;  // = dx_max
            if(res < tol)
                break;

            // 8) Begin Line Search
            {
                // Record Current State x to x_0

                auto compute_energy = [&](Float alpha) -> Float
                {
                    // Continuous Collision Detection => alpha
                    // Step Forward => x = x_0 + alpha * dx
                    // Compute New Energy => E
                    return 0.0;
                };

                // Compute Current Energy => E_0

                Float E0 = compute_energy(0.0);

                // Build Continuous Collision Pairs with alpha = 1.0
                Float alpha = 1.0;

                SizeT max_line_search_iter = 1000;  // now just hard code it
                while(max_line_search_iter--)       // Energy Test
                {
                    // Check some conditions
                    Float E               = compute_energy(alpha);
                    bool  energy_decrease = E < E0;  // Check Energy Decrease
                    bool  no_inversion    = true;    // Check Inversion

                    bool success = energy_decrease && no_inversion;
                    if(success)
                        break;

                    // If not success, then shrink alpha

                    alpha /= 2;
                    E = compute_energy(alpha);
                }
            }
        }

        // e. Update Velocity => v = (x - x_0) / dt
    }
}
}  // namespace uipc::backend::cuda
