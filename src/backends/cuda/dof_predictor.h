#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class DofPredictor : public SimSystem
{
  public:
    class PredictInfo
    {
      public:
        auto dt() const noexcept { return m_dt; }

      private:
        friend class DofPredictor;
        Float m_dt;
    };

    using ComputeVelocityInfo = PredictInfo;

    using SimSystem::SimSystem;

    void on_predict(SimSystem& system, std::function<void(PredictInfo&)>&& action);
    void on_compute_velocity(SimSystem& system,
                             std::function<void(ComputeVelocityInfo&)>&& action);

  protected:
    void do_build() override;

  private:
    void init();

    friend class SimEngine;
    void predict();           // only be called by SimEngine
    void compute_velocity();  // only be called by SimEngine

    SimActionCollection<void(PredictInfo&)>         m_on_predict;
    SimActionCollection<void(ComputeVelocityInfo&)> m_on_compute_velocity;

    Float m_dt;
};
}  // namespace uipc::backend::cuda
