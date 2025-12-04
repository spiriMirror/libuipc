#include <affine_body/affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
/**
 * @brief Constraint that reads external forces from geometry attributes
 *
 * This constraint reads "external_force" attributes from geometries
 * and stores them to body_id_to_external_force_raw (raw force data).
 *
 * The actual computation of M^{-1} * F (acceleration) is done by ABDExternalForceReporter
 * in the GlobalExternalForceManager lifecycle phase (after animator step).
 */
class ExternalForceConstraint final : public AffineBodyConstraint
{
  public:
    static constexpr U64 UID = 666;  // Same UID as AffineBodyExternalForce constitution

    using AffineBodyConstraint::AffineBodyConstraint;

    AffineBodyDynamics* affine_body_dynamics = nullptr;

    // Host-side buffers for collecting data before copying to device
    vector<Vector12> h_forces;
    vector<IndexT>   h_body_ids;

    virtual void do_build(BuildInfo& info) override
    {
        affine_body_dynamics = &require<AffineBodyDynamics>();
    }

    U64 get_uid() const noexcept override { return UID; }

    void do_init(AffineBodyAnimator::FilteredInfo& info) override
    {
        // Initial read of external forces
        do_step(info);
    }

    void do_step(AffineBodyAnimator::FilteredInfo& info) override
    {
        // Clear external forces buffer
        auto external_forces_raw = affine_body_dynamics->body_external_forces_raw();

        using namespace muda;
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(external_forces_raw.size(),
                   [forces = external_forces_raw.viewer().name("forces")] __device__(int i) mutable
                   {
                       forces(i).setZero();
                   });

        // Clear host buffers
        h_forces.clear();
        h_body_ids.clear();

        // Read external forces from geometry attributes
        auto geo_slots = world().scene().geometries();

        IndexT current_body_offset = 0;
        info.for_each(
            geo_slots,
            [&](geometry::SimplicialComplex& sc)
            {
                auto body_offset = sc.meta().find<IndexT>(builtin::backend_abd_body_offset);
                current_body_offset = body_offset->view().front();

                auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
                auto external_force = sc.instances().find<Vector12>("external_force");

                return zip(is_constrained->view(), external_force->view());
            },
            [&](const AffineBodyDynamics::ForEachInfo& I, auto&& values)
            {
                SizeT body_id = I.local_index() + current_body_offset;

                auto&& [is_constrained, force] = values;

                if(is_constrained)
                {
                    h_forces.push_back(force);
                    h_body_ids.push_back(body_id);
                }
            });

        // Copy from host to device
        if(!h_forces.empty())
        {
            // Scatter copy: for each (force, body_id) pair, write to external_forces_raw[body_id]
            muda::DeviceBuffer<Vector12> d_forces(h_forces.size());
            muda::DeviceBuffer<IndexT>   d_body_ids(h_body_ids.size());

            d_forces.view().copy_from(h_forces.data());
            d_body_ids.view().copy_from(h_body_ids.data());

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(h_forces.size(),
                       [forces_dst = external_forces_raw.viewer().name("dst"),
                        forces_src = d_forces.cviewer().name("src"),
                        body_ids   = d_body_ids.cviewer().name("ids")] __device__(int i) mutable
                       {
                           IndexT body_id = body_ids(i);
                           forces_dst(body_id) = forces_src(i);
                       });
        }
    }

    // No energy/gradient/hessian for external forces (they are applied directly in do_predict_dof)
    void do_report_extent(AffineBodyAnimator::ReportExtentInfo& info) override
    {
        // No contribution to energy/gradient/hessian
        info.energy_count(0);
        info.gradient_segment_count(0);
        info.hessian_block_count(0);
    }

    void do_compute_energy(AffineBodyAnimator::EnergyInfo& info) override
    {
        // No energy computation
    }

    void do_compute_gradient_hessian(AffineBodyAnimator::GradientHessianInfo& info) override
    {
        // No gradient/hessian computation
    }
};

REGISTER_SIM_SYSTEM(ExternalForceConstraint);
}  // namespace uipc::backend::cuda
