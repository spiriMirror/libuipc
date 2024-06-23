#include <gradient_hessian_computer.h>

namespace uipc::backend::cuda
{
void GradientHessianComputer::do_build()
{
    on_init_scene([this] { this->init(); });
}

void GradientHessianComputer::on_compute_gradient_hessian(std::function<void(ComputeInfo&)>&& compute_gradient_hessian)
{
    check_state(SimEngineState::BuildSystems, "on_compute_gradient_hessian()");
    m_on_compute_gradient_hessian.emplace_back(std::move(compute_gradient_hessian));
}

void GradientHessianComputer::init()
{
    auto scene = world().scene();
    m_dt       = scene.info()["dt"];
}

void GradientHessianComputer::compute_gradient_hessian()
{
    ComputeInfo compute_info{this};
    for(auto& action : m_on_compute_gradient_hessian)
    {
        action(compute_info);
    }
}

Float GradientHessianComputer::ComputeInfo::dt() const noexcept
{
    return m_impl->m_dt;
}
}  // namespace uipc::backend::cuda
