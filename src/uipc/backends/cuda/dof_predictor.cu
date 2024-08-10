#include <dof_predictor.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DoFPredictor);

void DoFPredictor::do_build()
{
    auto scene = world().scene();
    m_dt       = scene.info()["dt"];
    on_init_scene([this]() { init(); });
}

void DoFPredictor::on_predict(SimSystem& system, std::function<void(PredictInfo&)>&& action)
{
    m_on_predict.register_action(system, std::move(action));
}

void DoFPredictor::on_compute_velocity(SimSystem& system,
                                       std::function<void(PredictInfo&)>&& action)
{
    m_on_compute_velocity.register_action(system, std::move(action));
}

void DoFPredictor::init()
{
    [[maybe_unuse]] m_on_predict.view();
    [[maybe_unuse]] m_on_compute_velocity.view();
}

void DoFPredictor::predict()
{
    for(auto& action : m_on_predict.view())
    {
        PredictInfo info;
        info.m_dt = m_dt;
        action(info);
    }
}
void DoFPredictor::compute_velocity()
{
    for(auto& action : m_on_compute_velocity.view())
    {
        ComputeVelocityInfo info;
        info.m_dt = m_dt;
        action(info);
    }
}
}  // namespace uipc::backend::cuda
