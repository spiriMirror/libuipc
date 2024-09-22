#include <gradient_hessian_computer.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GradientHessianComputer);

void GradientHessianComputer::do_build()
{
    on_init_scene([this] { this->init(); });
}

void GradientHessianComputer::on_compute_gradient_hessian(
    SimSystem& system, std::function<void(ComputeInfo&)>&& compute_gradient_hessian)
{
    check_state(SimEngineState::BuildSystems, "on_compute_gradient_hessian()");
    m_on_compute_gradient_hessian.register_action(system, std::move(compute_gradient_hessian));
}

void GradientHessianComputer::init()
{
    auto scene = world().scene();
    m_dt       = scene.info()["dt"];

    [[maybe_unuse]] m_on_compute_gradient_hessian.view();
}

void GradientHessianComputer::compute_gradient_hessian()
{
    ComputeInfo compute_info{this};
    for(auto& action : m_on_compute_gradient_hessian.view())
    {
        action(compute_info);
    }
}

Float GradientHessianComputer::ComputeInfo::dt() const noexcept
{
    return m_impl->m_dt;
}
}  // namespace uipc::backend::cuda
