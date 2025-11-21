#pragma once
#include <sim_system.h>
#include <affine_body/affine_body_dynamics.h>

namespace uipc::backend::cuda
{
class ABDLinearSubsystem;
class AffineBodyAnimator;

class AffineBodyExtraConstitution : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class Impl
    {
      public:
        void init(U64 uid, backend::WorldVisitor& world);

        AffineBodyDynamics*                 affine_body_dynamics = nullptr;
        vector<AffineBodyDynamics::GeoInfo> geo_infos;
        AffineBodyDynamics::Impl&           abd() noexcept
        {
            return affine_body_dynamics->m_impl;
        }
    };

    class BuildInfo
    {
      public:
    };

    class FilteredInfo
    {
      public:
        FilteredInfo(Impl* impl) noexcept
            : m_impl(impl)
        {
        }

        span<const AffineBodyDynamics::GeoInfo> geo_infos() const noexcept;

        template <typename ViewGetter, typename ForEach>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots,
                      ViewGetter&&                    view_getter,
                      ForEach&&                       for_each_action);

        template <typename ForEach>
        void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEach&& for_each_action);

        span<const Vector12> qs() noexcept;
        span<const Vector12> q_vs() noexcept;

      private:
        Impl* m_impl = nullptr;
    };

    class BaseInfo
    {
      public:
        BaseInfo(AffineBodyDynamics::Impl* impl, Float dt)
            : m_impl(impl)
            , m_dt(dt)
        {
        }

        Float dt() const noexcept;

        muda::CBufferView<Vector12> qs() const noexcept;
        muda::CBufferView<Vector12> q_bars() const noexcept;

      protected:
        AffineBodyDynamics::Impl* m_impl = nullptr;
        Float                     m_dt;
    };

    class ComputeEnergyInfo : public BaseInfo
    {
      public:
        ComputeEnergyInfo(AffineBodyDynamics::Impl* impl, Float dt, muda::BufferView<Float> energies)
            : BaseInfo(impl, dt)
            , m_energies(energies)
        {
        }

        auto energies() const noexcept { return m_energies; }

      private:
        muda::BufferView<Float> m_energies;
    };

    class ComputeGradientHessianInfo : public BaseInfo
    {
      public:
        ComputeGradientHessianInfo(AffineBodyDynamics::Impl*      impl,
                                   Float                          dt,
                                   muda::BufferView<Vector12>     gradients,
                                   muda::BufferView<Matrix12x12>  hessians)
            : BaseInfo(impl, dt)
            , m_gradients(gradients)
            , m_hessians(hessians)
        {
        }

        auto gradients() const noexcept { return m_gradients; }
        auto hessians() const noexcept { return m_hessians; }

      private:
        muda::BufferView<Vector12>    m_gradients;
        muda::BufferView<Matrix12x12> m_hessians;
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                                        = 0;
    virtual U64  get_uid() const noexcept                                         = 0;
    virtual void do_init(FilteredInfo& info)                                      = 0;
    virtual void do_step(FilteredInfo& info) {}  // optional: update data from scene each substep
    virtual void do_compute_energy(ComputeEnergyInfo& info)                       = 0;
    virtual void do_compute_gradient_hessian(ComputeGradientHessianInfo& info)    = 0;

    friend class AffineBodyDynamics;
    friend class ABDLinearSubsystem;
    span<const AffineBodyDynamics::GeoInfo> geo_infos() const noexcept;

  private:
    friend class AffineBodyDynamics;
    friend class ABDLinearSubsystem;
    friend class AffineBodyAnimator;
    void do_build() override final;
    void init();  // only be called by AffineBodyDynamics
    void step();  // only be called by AffineBodyAnimator at each substep

    // These will be called by AffineBodyDynamics during energy/gradient computation
    void compute_energy(AffineBodyDynamics::Impl& abd_impl, Float dt, muda::BufferView<Float> energies);
    void compute_gradient_hessian(AffineBodyDynamics::Impl&     abd_impl,
                                   Float                         dt,
                                   muda::BufferView<Vector12>    gradients,
                                   muda::BufferView<Matrix12x12> hessians);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda

#include "details/affine_body_extra_constitution.inl"
