#pragma once
#include <active_set_system/al_stiffness_estimator.h>
#include <affine_body/affine_body_vertex_reporter.h>

namespace uipc::backend::cuda
{

class ABDALStiffnessEstimator final : public ALStiffnessEstimator
{
  public:
    using ALStiffnessEstimator::ALStiffnessEstimator;

    class Impl
    {
      public:
        SimSystemSlot<AffineBodyDynamics>       affine_body_dynamics;
        SimSystemSlot<AffineBodyVertexReporter> vertex_reporter;
        Float                                   mu_scale_abd = 0.0;

        void estimate_mu(EstimateInfo& info);
    };

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_estimate_mu(EstimateInfo& info) override;

  private:
    Impl m_impl;
};

}  // namespace uipc::backend::cuda
