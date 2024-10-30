#pragma once
#include <finite_element/finite_element_energy_producer.h>

namespace uipc::backend::cuda
{
class FiniteElementKinetic final : public FiniteElementEnergyProducer
{
  public:
    using FiniteElementEnergyProducer::FiniteElementEnergyProducer;

    class Impl
    {
      public:
        void compute_energy(ComputeEnergyInfo& info);
        void compute_gradient_hessian(ComputeGradientHessianInfo& info);

        FiniteElementMethod*       finite_element_method = nullptr;
        FiniteElementMethod::Impl& fem() const noexcept
        {
            return finite_element_method->m_impl;
        }
    };

  private:
    // Inherited via FiniteElementEnergyProducer
    virtual void do_build(BuildInfo& info) override;
    virtual void do_report_extent(ReportExtentInfo& info) override;
    virtual void do_compute_energy(ComputeEnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
