#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class TimeIntegrator;

class TimeIntegratorManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class PredictDofInfo
    {
      public:
        auto dt() const noexcept { return m_dt; }

      private:
        friend class TimeIntegratorManager;
        Float m_dt;
    };

    class UpdateStateInfo
    {
      public:
        auto dt() const noexcept { return m_dt; };

      private:
        friend class TimeIntegratorManager;
        Float m_dt;
    };

    class Impl
    {
      public:
        SimSystemSlotCollection<TimeIntegrator> time_integrators;
        Float                                   dt = 0.0;
    };

  private:
    friend class SimEngine;
    virtual void do_build() override;
    void         init();

    void predict_dof();
    void update_state();

    friend class TimeIntegrator;
    void add_integrator(TimeIntegrator* integrator);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda