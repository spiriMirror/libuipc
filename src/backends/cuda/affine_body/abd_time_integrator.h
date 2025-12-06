#pragma once
#include <affine_body/affine_body_dynamics.h>
#include <time_integrator/time_integrator.h>

namespace uipc::backend::cuda
{
class ABDTimeIntegrator : public TimeIntegrator
{
  public:
    using TimeIntegrator::TimeIntegrator;

    class Impl
    {
      public:
        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
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

        auto q_vs() const noexcept
        {
            return m_impl->affine_body_dynamics->q_vs();
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

        auto external_force_accs() const noexcept
        {
            return m_impl->affine_body_dynamics->body_external_force_accs();
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

        auto q_tildes() const noexcept
        {
            return m_impl->affine_body_dynamics->m_impl.body_id_to_q_tilde.view();
        }

        auto q_prevs() const noexcept
        {
            return m_impl->affine_body_dynamics->m_impl.body_id_to_q_prev.view();
        }

      private:
        TimeIntegrator::PredictDofInfo* base_info = nullptr;
    };

    class UpdateVelocityInfo : public BaseInfo
    {
      public:
        UpdateVelocityInfo(Impl* impl, TimeIntegrator::UpdateVelocityInfo* base_info)
            : BaseInfo(impl)
            , base_info(base_info)
        {
        }

        auto dt() const noexcept { return base_info->dt(); }

        auto q_vs() const noexcept
        {
            return m_impl->affine_body_dynamics->m_impl.body_id_to_q_v.view();
        }

      private:
        TimeIntegrator::UpdateVelocityInfo* base_info = nullptr;
    };


  protected:
    virtual void do_build(BuildInfo& info) = 0;

    virtual void do_init(InitInfo& info)                   = 0;
    virtual void do_predict_dof(PredictDofInfo& info)      = 0;
    virtual void do_update_state(UpdateVelocityInfo& info) = 0;

  private:
    void do_build(TimeIntegrator::BuildInfo& info) override;

    void do_init(TimeIntegrator::InitInfo& info) override;
    void do_predict_dof(TimeIntegrator::PredictDofInfo& info) override;
    void do_update_state(TimeIntegrator::UpdateVelocityInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
