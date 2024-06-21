#include <gradient_hessian_computer.h>

namespace uipc::backend::cuda
{
void GradientHessianComputer::on_compute_gradient_hessian(
    std::function<void(const ComputeInfo&)>&& compute_gradient_hessian)
{
    check_state(SimEngineState::BuildSystems, "on_compute_gradient_hessian()");
    m_on_compute_gradient_hessian.emplace_back(std::move(compute_gradient_hessian));
}
void GradientHessianComputer::compute_gradient_hessian()
{
    ComputeInfo compute_info;
    for(auto& action : m_on_compute_gradient_hessian)
    {
        action(compute_info);
    }
}
}  // namespace uipc::backend::cuda
