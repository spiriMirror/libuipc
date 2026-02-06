#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/abd_line_search_reporter.h>
#include <affine_body/abd_linear_subsystem.h>

namespace uipc::backend::cuda
{
class AffineBodyConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    U64 uid() const;

    class BuildInfo
    {
      public:
    };

    class Impl
    {
      public:
        SimSystemSlot<AffineBodyDynamics> affine_body_dynamics;
        AffineBodyDynamics::Impl&         abd() noexcept
        {
            return affine_body_dynamics->m_impl;
        }
    };

    class BaseInfo
    {
      public:
        BaseInfo(Impl* impl, IndexT index) noexcept
            : m_impl(impl)
            , m_index(index)
        {
        }

        auto is_fixed() const noexcept
        {
            return m_impl->affine_body_dynamics->body_is_fixed();
        }

        muda::CBufferView<Vector12> qs() const noexcept;

        muda::CBufferView<Vector12> q_prevs() const noexcept;

        muda::CBufferView<Float> volumes() const noexcept;

      protected:
        Impl*  m_impl  = nullptr;
        IndexT m_index = -1;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(Impl* impl, IndexT index, ABDLineSearchReporter::ComputeEnergyInfo* base_info) noexcept
            : BaseInfo(impl, index)
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
        ComputeGradientHessianInfo(Impl*  impl,
                                   IndexT index,
                                   ABDLinearSubsystem::ComputeGradientHessianInfo* base_info) noexcept
            : BaseInfo(impl, index)
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
    virtual U64  get_uid() const                                 = 0;
    virtual void do_build(BuildInfo& info)                       = 0;
    virtual void do_init(AffineBodyDynamics::FilteredInfo& info) = 0;
    virtual void do_compute_energy(ComputeEnergyInfo& info)      = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info) = 0;

  private:
    friend class AffineBodyDynamics;
    friend class ABDLineSearchReporter;
    friend class ABDLinearSubsystem;

    virtual void do_build() override final;

    void init(AffineBodyDynamics::FilteredInfo& info);
    void compute_energy(ABDLineSearchReporter::ComputeEnergyInfo& info);
    void compute_gradient_hessian(ABDLinearSubsystem::ComputeGradientHessianInfo& info);

    IndexT m_index = -1;
    Impl   m_impl;
};
}  // namespace uipc::backend::cuda
