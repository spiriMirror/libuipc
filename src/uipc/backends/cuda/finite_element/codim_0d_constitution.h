#pragma once
#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
class Codim0DConstitution : public FiniteElementConstitution
{
  public:
    using FiniteElementConstitution::FiniteElementConstitution;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_retrieve(FiniteElementMethod::Codim0DFilteredInfo& info) = 0;

  private:
    friend class FiniteElementMethod;
    void retrieve(FiniteElementMethod::Codim0DFilteredInfo& info);
    virtual void do_build(FiniteElementConstitution::BuildInfo& info) override final;
    virtual IndexT get_dimension() const override final;
    virtual void do_compute_energy(FiniteElementMethod::ComputeEnergyInfo& info) override final;
    virtual void do_compute_gradient_hessian(
        FiniteElementMethod::ComputeGradientHessianInfo& info) override final;
};
}  // namespace uipc::backend::cuda
