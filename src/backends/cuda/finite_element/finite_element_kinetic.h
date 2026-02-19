#pragma once
#include <sim_system.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/fem_line_search_reporter.h>
#include <finite_element/fem_linear_subsystem.h>

namespace uipc::backend::cuda
{
class FiniteElementKinetic : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementMethod> finite_element_method;
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
            return m_impl->finite_element_method->is_fixed();
        }

        auto is_dynamic() const noexcept
        {
            return m_impl->finite_element_method->is_dynamic();
        }

        auto x_bars() const noexcept
        {
            return m_impl->finite_element_method->x_bars();
        }

        auto xs() const noexcept { return m_impl->finite_element_method->xs(); }

        auto x_tildes() const noexcept
        {
            return m_impl->finite_element_method->x_tildes();
        }

        auto x_prevs() const noexcept
        {
            return m_impl->finite_element_method->x_prevs();
        }

        auto masses() const noexcept
        {
            return m_impl->finite_element_method->masses();
        }


      protected:
        Impl* m_impl = nullptr;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, FEMLineSearchReporter::ComputeEnergyInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto energies() const noexcept { return base_info->energies(); }
        auto dt() const noexcept { return base_info->dt(); }

      private:
        FEMLineSearchReporter::ComputeEnergyInfo* base_info = nullptr;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   FEMLinearSubsystem::ComputeGradientHessianInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto gradient_only() const noexcept { return base_info->gradient_only(); }
        auto gradients() const noexcept { return base_info->gradients(); }
        auto hessians() const noexcept { return base_info->hessians(); }
        auto dt() const noexcept { return base_info->dt(); }

      private:
        FEMLinearSubsystem::ComputeGradientHessianInfo* base_info = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info)                  = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    virtual void do_build() override final;

    friend class FEMLineSearchReporter;
    void compute_energy(FEMLineSearchReporter::ComputeEnergyInfo& info);

    friend class FEMLinearSubsystem;
    void compute_gradient_hessian(FEMLinearSubsystem::ComputeGradientHessianInfo& info);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
