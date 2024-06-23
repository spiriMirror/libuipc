#pragma once
#include <sim_system.h>
#include <uipc/common/list.h>
#include <functional>
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

    void on_predict(std::function<void(PredictInfo&)>&& action);
    void on_compute_velocity(std::function<void(PredictInfo&)>&& action);

  protected:
    void do_build() override;

  private:
    void find_predict_info();

    friend class SimEngine;
    void predict();           // only be called by SimEngine
    void compute_velocity();  // only be called by SimEngine

    list<std::function<void(PredictInfo&)>> m_on_predict;
    list<std::function<void(PredictInfo&)>> m_on_compute_velocity;

    PredictInfo m_predict_info;
};
}  // namespace uipc::backend::cuda
