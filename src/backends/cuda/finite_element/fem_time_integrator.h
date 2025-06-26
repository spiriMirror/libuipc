#pragma once
#include <finite_element/finite_element_method.h>
#include <time_integrator/time_integrator.h>

namespace uipc::backend::cuda
{
class FEMTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    class Impl
    {
      public:
        SimSystemSlot<FiniteElementMethod> finite_element_method;
    };

    class BuildInfo
    {
      public:
    };

    class InitInfo
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

        auto vs() const noexcept { return m_impl->finite_element_method->vs(); }

        muda::BufferView<Vector3> x_tildes() const noexcept
        {
            return m_impl->finite_element_method->m_impl.x_tildes.view();
        }

        auto x_prevs() const noexcept
        {
            return m_impl->finite_element_method->x_prevs();
        }

        auto masses() const noexcept
        {
            return m_impl->finite_element_method->masses();
        }

        auto gravities() const noexcept
        {
            return m_impl->finite_element_method->gravities();
        }

      protected:
        Impl* m_impl = nullptr;
    };

    class PredictDofInfo : public BaseInfo
    {
      public:
        PredictDofInfo(Impl* impl, TimeIntegrator::PredictDofInfo* base_info)
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto dt() const noexcept { return base_info->dt(); }


      private:
        TimeIntegrator::PredictDofInfo* base_info = nullptr;
    };

    class UpdateStateInfo : public BaseInfo
    {
      public:
        UpdateStateInfo(Impl* impl, TimeIntegrator::UpdateStateInfo* base_info)
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto dt() const noexcept { return base_info->dt(); }

        auto x_prevs() noexcept
        {
            return m_impl->finite_element_method->m_impl.x_prevs.view();
        }

        auto vs() noexcept
        {
            return m_impl->finite_element_method->m_impl.vs.view();
        }

      private:
        TimeIntegrator::UpdateStateInfo* base_info = nullptr;
    };


  protected:
    virtual void do_build(BuildInfo& info) = 0;

    virtual void do_init(InitInfo& info)                = 0;
    virtual void do_predict_dof(PredictDofInfo& info)   = 0;
    virtual void do_update_state(UpdateStateInfo& info) = 0;

  private:
    void do_build(TimeIntegrator::BuildInfo& info) override;

    void do_init(TimeIntegrator::InitInfo& info) override;
    void do_predict_dof(TimeIntegrator::PredictDofInfo& info) override;
    void do_update_state(TimeIntegrator::UpdateStateInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
