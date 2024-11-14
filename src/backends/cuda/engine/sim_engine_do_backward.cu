#include <sim_engine.h>
#include <diff_sim/global_diff_sim_manager.h>
#include <linear_system/global_linear_system.h>
namespace uipc::backend::cuda
{
void SimEngine::do_backward()
{
    try
    {
        if(m_global_diff_sim_manager)
        {
            if(m_last_solved_frame != m_current_frame)
            {
                throw SimEngineException(  //
                    fmt::format("The current frame({}) is not solved yet. The last solved frame is {}. "
                                "DiffSim depends on the simlulation result of the current frame. "
                                "Make sure the current frame is solved (rather than recovered) before calling `backward()`.",
                                m_current_frame,
                                m_last_solved_frame));
            }
            //tex: 1) prepare the global hessian matrix for current frame
            //
            //$$
            // \frac{\partial^2 E}{\partial X^{[i]} \partial X^{[i]}}
            //$$
            // where $^{[i]}$ is the i-th frame
            m_global_linear_system->prepare_hessian();

            //tex: 2) assemble the $\frac{\partial G^{[i]}}{\partial P}$ and $H^{[i]}$ for current frame
            //
            //
            //$$
            // \frac{\partial G^{[i]}}{\partial P} := \frac{\partial^2 E}{\partial X^{[i]} \partial P}
            //$$
            //
            //
            //$$
            //H^{[i]} :=
            //\begin{bmatrix}
            // \frac{\partial^2 E}{\partial X^{[i]} \partial X^{[1]}} & \cdots & \frac{\partial^2 E}{\partial X^{[i]} \partial X^{[i]}} \\
            //\end{bmatrix}
            //$$
            m_global_diff_sim_manager->assemble();
        }
        else
        {
            spdlog::warn("DiffSim is disabled. Backward does nothing");
        }
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("SimEngine Backward Error: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}

}  // namespace uipc::backend::cuda
