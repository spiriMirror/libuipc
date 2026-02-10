#pragma once
#include <sim_system.h>
#include <affine_body/inter_affine_body_animator.h>

namespace uipc::backend::cuda
{
class InterAffineBodyConstraint : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    U64 uid() const noexcept;

  protected:
    virtual void do_build(BuildInfo& info)                            = 0;
    virtual U64  get_uid() const noexcept                             = 0;
    virtual void do_init(InterAffineBodyAnimator::FilteredInfo& info) = 0;
    virtual void do_step(InterAffineBodyAnimator::FilteredInfo& info) = 0;
    virtual void do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info) = 0;
    virtual void do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info) = 0;
    virtual void do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info) = 0;


    template <typename ForEachGeometry>
    void for_each(span<S<geometry::GeometrySlot>> geo_slots, ForEachGeometry&& for_every_geometry)
    {
        InterAffineBodyAnimator::_for_each(geo_slots,
                                           animated_inter_geo_info(),
                                           std::forward<ForEachGeometry>(for_every_geometry));
    }

    span<const InterAffineBodyAnimator::AnimatedInterGeoInfo> animated_inter_geo_info() const noexcept;

  private:
    friend class InterAffineBodyAnimator;
    virtual void do_build() override final;

    void init(InterAffineBodyAnimator::FilteredInfo& info);
    void step(InterAffineBodyAnimator::FilteredInfo& info);
    void report_extent(InterAffineBodyAnimator::ReportExtentInfo& info);
    void compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info);
    void compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info);

    SizeT                                  m_index = ~0ull;
    SimSystemSlot<InterAffineBodyAnimator> m_animator;
};
}  // namespace uipc::backend::cuda
