#include <dof_predictor.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DoFPredictor);

void DoFPredictor::do_build()
{
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
    m_on_predict.init();
    m_on_compute_velocity.init();

    auto scene        = world().scene();
    m_predict_info.dt = scene.info()["dt"];
    spdlog::info("Find predict info: dt = {}", m_predict_info.dt);
}

void DoFPredictor::predict()
{
    for(auto& action : m_on_predict.view())
        action(m_predict_info);
}
void DoFPredictor::compute_velocity()
{
    for(auto& action : m_on_compute_velocity.view())
        action(m_predict_info);
}
}  // namespace uipc::backend::cuda
