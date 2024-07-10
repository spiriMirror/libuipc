#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class DoFPredictor : public SimSystem
{
  public:
    class PredictInfo
    {
      public:
        Float dt;
    };

    using SimSystem::SimSystem;

    void on_predict(SimSystem& system, std::function<void(PredictInfo&)>&& action);
    void on_compute_velocity(SimSystem& system, std::function<void(PredictInfo&)>&& action);

  protected:
    void do_build() override;

  private:
    void init();

    friend class SimEngine;
    void predict();           // only be called by SimEngine
    void compute_velocity();  // only be called by SimEngine

    SimActionCollection<void(PredictInfo&)> m_on_predict;
    SimActionCollection<void(PredictInfo&)> m_on_compute_velocity;

    PredictInfo m_predict_info;
};
}  // namespace uipc::backend::cuda
