#include <dof_predictor.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(DoFPredictor);

void DoFPredictor::do_build()
{
    on_init_scene([this]() { find_predict_info(); });
}

void DoFPredictor::on_predict(std::function<void(PredictInfo&)>&& action)
{
    m_on_predict.push_back(std::move(action));
}

void DoFPredictor::on_compute_velocity(std::function<void(PredictInfo&)>&& action)
{
    m_on_compute_velocity.push_back(std::move(action));
}

void DoFPredictor::find_predict_info()
{
    auto scene        = world().scene();
    m_predict_info.dt = scene.info()["dt"];
    spdlog::info("Find predict info: dt = {}", m_predict_info.dt);
}

void DoFPredictor::predict()
{
    for(auto& action : m_on_predict)
        action(m_predict_info);
}
void DoFPredictor::compute_velocity()
{
    for(auto& action : m_on_compute_velocity)
        action(m_predict_info);
}
}  // namespace uipc::backend::cuda
