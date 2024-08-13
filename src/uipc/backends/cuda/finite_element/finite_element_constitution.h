#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
class FiniteElementConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    U64    constitution_uid() const;
    IndexT dimension() const;

    class BuildInfo
    {
      public:
    };

  protected:
    virtual U64    get_constitution_uid() const = 0;
    virtual IndexT get_dimension() const        = 0;
    virtual void   do_build(BuildInfo& info)    = 0;
    virtual void do_compute_energy(FiniteElementMethod::ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info) = 0;

  private:
    friend class FiniteElementMethod;
    friend class FEMLineSearchReporter;
    friend class FEMGradientHessianComputer;
    friend class Codim0DConstitution;
    friend class Codim1DConstitution;
    friend class Codim2DConstitution;
    friend class FEM3DConstitution;

    virtual void do_build() override final;

    void compute_energy(FiniteElementMethod::ComputeEnergyInfo& info);
    void compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info);

    SizeT                      m_index        = ~0ull;
    SizeT                      m_index_in_dim = ~0ull;
    FiniteElementMethod*       m_fem          = nullptr;
    FiniteElementMethod::Impl& fem() { return m_fem->m_impl; }
};
}  // namespace uipc::backend::cuda
