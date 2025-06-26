#pragma once
#include <finite_element/finite_element_energy_producer.h>

namespace uipc::backend::cuda
{
class FiniteElementKinetic : public FiniteElementEnergyProducer
{
  public:
    using FiniteElementEnergyProducer::FiniteElementEnergyProducer;

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
        ComputeEnergyInfo(Impl* impl, FiniteElementEnergyProducer::ComputeEnergyInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto energies() const noexcept { return base_info->energies(); }
        auto dt() const noexcept { return base_info->dt(); }

      private:
        FiniteElementEnergyProducer::ComputeEnergyInfo* base_info = nullptr;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(Impl* impl,
                                   FiniteElementEnergyProducer::ComputeGradientHessianInfo* base_info) noexcept
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto gradients() const noexcept { return base_info->gradients(); }
        auto hessians() const noexcept { return base_info->hessians(); }
        auto dt() const noexcept { return base_info->dt(); }

      private:
        FiniteElementEnergyProducer::ComputeGradientHessianInfo* base_info = nullptr;
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_report_extent(FiniteElementEnergyProducer::ReportExtentInfo& info) override;
    virtual void do_compute_energy(ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;


  private:
    virtual void do_build(FiniteElementEnergyProducer::BuildInfo& info) override;
    virtual void do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(
        FiniteElementEnergyProducer::ComputeGradientHessianInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
