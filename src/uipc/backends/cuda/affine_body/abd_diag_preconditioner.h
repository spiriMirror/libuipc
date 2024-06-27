#pragma once
#include <linear_system/local_preconditioner.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class ABDDiagPreconditioner : public LocalPreconditioner
{
  public:
    using LocalPreconditioner::LocalPreconditioner;

    class Impl
    {
      public:
        void assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info);
        void apply(GlobalLinearSystem::ApplyPreconditionerInfo& info);

        AffineBodyDynamics*       affine_body_dynamics = nullptr;
        AffineBodyDynamics::Impl& abd() { return affine_body_dynamics->m_impl; }

        muda::DeviceBuffer<Matrix12x12> inv_diag;
    };

  protected:
    virtual void do_build() override;

  private:
    Impl m_impl;
    // Inherited via LocalPreconditioner
    virtual void do_assemble(GlobalLinearSystem::LocalPreconditionerAssemblyInfo& info) override;
    virtual void do_apply(GlobalLinearSystem::ApplyPreconditionerInfo& info) override;
};
}  // namespace uipc::backend::cuda
