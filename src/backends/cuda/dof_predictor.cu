#include <dof_predictor.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DofPredictor);

void DofPredictor::do_build()
{
    auto scene = world().scene();
    m_dt       = scene.info()["dt"];
    on_init_scene([this]() { init(); });
}

void DofPredictor::on_predict(SimSystem& system, std::function<void(PredictInfo&)>&& action)
{
    m_on_predict.register_action(system, std::move(action));
}

void DofPredictor::on_compute_velocity(SimSystem& system,
                                       std::function<void(PredictInfo&)>&& action)
{
    m_on_compute_velocity.register_action(system, std::move(action));
}

void DofPredictor::init()
{
    [[maybe_unuse]] m_on_predict.view();
    [[maybe_unuse]] m_on_compute_velocity.view();
}

void DofPredictor::predict()
{
    for(auto& action : m_on_predict.view())
    {
        PredictInfo info;
        info.m_dt = m_dt;
        action(info);
    }
}
void DofPredictor::compute_velocity()
{
    for(auto& action : m_on_compute_velocity.view())
    {
        ComputeVelocityInfo info;
        info.m_dt = m_dt;
        action(info);
    }
}
}  // namespace uipc::backend::cuda
