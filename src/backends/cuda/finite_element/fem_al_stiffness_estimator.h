#pragma once
#include <active_set_system/al_stiffness_estimator.h>
#include <finite_element/finite_element_vertex_reporter.h>

namespace uipc::backend::cuda
{

class FEMALStiffnessEstimator final : public ALStiffnessEstimator
{
  public:
    using ALStiffnessEstimator::ALStiffnessEstimator;

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementVertexReporter> vertex_reporter;
        SimSystemSlot<FiniteElementMethod>         finite_element_method;
        Float                                      mu_scale_fem = 0.0;

        void estimate_mu(EstimateInfo& info);
    };

  protected:
    virtual void do_build(BuildInfo& info) override;
    virtual void do_estimate_mu(EstimateInfo& info) override;

  private:
    Impl m_impl;
};

}  // namespace uipc::backend::cuda
