#pragma once
#include <finite_element/finite_element_constraint.h>

namespace uipc::backend::cuda
{
/**
 * @brief Constraint that reads per-vertex external forces from geometry attributes
 *
 * This constraint reads "external_force" (Vector3) attributes from vertices
 * and stores them inside its local buffers. No energy/gradient/hessian --
 * forces are applied through x_tilde in the time integrator.
 */
class FiniteElementExternalVertexForceConstraint final : public FiniteElementConstraint
{
  public:
    using FiniteElementConstraint::FiniteElementConstraint;

    static constexpr U64 UID = 671;

    class Impl
    {
      public:
        vector<Vector3>             h_forces;
        vector<IndexT>              h_vertex_ids;
        muda::DeviceBuffer<Vector3> forces;
        muda::DeviceBuffer<IndexT>  vertex_ids;

        void step(backend::WorldVisitor& world, FiniteElementAnimator::FilteredInfo& info);
    };

    muda::CBufferView<Vector3> forces() const noexcept;
    muda::CBufferView<IndexT>  vertex_ids() const noexcept;

  private:
    virtual void do_build(BuildInfo& info) override;
    virtual U64  get_uid() const noexcept override;
    virtual void do_init(FiniteElementAnimator::FilteredInfo& info) override;
    virtual void do_step(FiniteElementAnimator::FilteredInfo& info) override;
    virtual void do_report_extent(FiniteElementAnimator::ReportExtentInfo& info) override;
    virtual void do_compute_energy(FiniteElementAnimator::ComputeEnergyInfo& info) override;
    virtual void do_compute_gradient_hessian(FiniteElementAnimator::ComputeGradientHessianInfo& info) override;

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
