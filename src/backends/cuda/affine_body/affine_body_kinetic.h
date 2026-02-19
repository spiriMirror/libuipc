#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/abd_linear_subsystem.h>

namespace uipc::backend::cuda
{
class AffineBodyKinetic : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
    };

    class BuildInfo
    {
      public:
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        auto is_fixed() const noexcept
        {
            return m_impl->affine_body_dynamics->body_is_fixed();
        }

        auto is_dynamic() const noexcept
        {
            return m_impl->affine_body_dynamics->body_is_dynamic();
        }

        auto Js() const noexcept { return m_impl->affine_body_dynamics->Js(); }

        auto qs() const noexcept { return m_impl->affine_body_dynamics->qs(); }

        auto q_tildes() const noexcept
        {
            return m_impl->affine_body_dynamics->q_tildes();
        }

        auto q_prevs() const noexcept
        {
            return m_impl->affine_body_dynamics->q_prevs();
        }

        auto masses() const noexcept
        {
            return m_impl->affine_body_dynamics->body_masses();
        }

        auto gravities() const noexcept
        {
            return m_impl->affine_body_dynamics->body_gravities();
        }

        auto external_kinetic() const noexcept
        {
            return m_impl->affine_body_dynamics->body_external_kinetic();
        }

      protected:
        Impl* m_impl = nullptr;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, ABDLineSearchReporter::ComputeEnergyInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto energies() const noexcept { return base_info->energies(); }

        auto dt() const noexcept { return base_info->dt(); }

      private:
        ABDLineSearchReporter::ComputeEnergyInfo* base_info = nullptr;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   ABDLinearSubsystem::ComputeGradientHessianInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto gradient_only() const noexcept { return base_info->gradient_only(); }
        auto gradients() const noexcept { return base_info->gradients(); }
        auto hessians() const noexcept { return base_info->hessians(); }
        auto dt() const noexcept { return base_info->dt(); }

      private:
        ABDLinearSubsystem::ComputeGradientHessianInfo* base_info = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;


  private:
    virtual void do_build() override final;

    friend class ABDLineSearchReporter;
    void compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info);
    friend class AffineBodyDynamics;
    friend class ABDLinearSubsystem;
    void compute_gradient_hessian(ABDLinearSubsystem::ComputeGradientHessianInfo& info);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
