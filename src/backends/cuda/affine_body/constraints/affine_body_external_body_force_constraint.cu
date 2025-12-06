#include <affine_body/constraints/affine_body_external_body_force_constraint.h>
#include <affine_body/affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyExternalBodyForceConstraint);

void AffineBodyExternalBodyForceConstraint::do_build(BuildInfo& info)
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
}

U64 AffineBodyExternalBodyForceConstraint::get_uid() const noexcept
{
    return UID;
}

void AffineBodyExternalBodyForceConstraint::do_init(AffineBodyAnimator::FilteredInfo& info)
{
    // Initial read of external forces
    do_step(info);
}

muda::CBufferView<Vector12> AffineBodyExternalBodyForceConstraint::forces() const noexcept
{
    return m_impl.forces.view();
}

muda::CBufferView<IndexT> AffineBodyExternalBodyForceConstraint::body_ids() const noexcept
{
    return m_impl.body_ids.view();
}

void AffineBodyExternalBodyForceConstraint::Impl::step(backend::WorldVisitor& world,
                                                       AffineBodyAnimator::FilteredInfo& info)
{
    // Clear host buffers
    h_forces.clear();
    h_body_ids.clear();
    // Read external forces from geometry attributes
    auto   geo_slots           = world.scene().geometries();
    IndexT current_body_offset = 0;
    info.for_each(
        geo_slots,
        [&](geometry::SimplicialComplex& sc)
        {
            auto body_offset = sc.meta().find<IndexT>(builtin::backend_abd_body_offset);
            UIPC_ASSERT(body_offset, "`backend_abd_body_offset` attribute not found in geometry simplicial complex");
            current_body_offset = body_offset->view().front();
            auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
            UIPC_ASSERT(is_constrained, "`is_constrained` attribute not found in geometry simplicial complex");
            auto external_force = sc.instances().find<Vector12>("external_force");
            UIPC_ASSERT(external_force, "`external_force` attribute not found in geometry simplicial complex");
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
        forces.copy_from(h_forces);
        body_ids.copy_from(h_body_ids);
    }
}

void AffineBodyExternalBodyForceConstraint::do_step(AffineBodyAnimator::FilteredInfo& info)
{
    m_impl.step(world(), info);
}

// No energy/gradient/hessian for external forces (they are applied directly in do_predict_dof)
void AffineBodyExternalBodyForceConstraint::do_report_extent(AffineBodyAnimator::ReportExtentInfo& info)
{
    // No contribution to energy/gradient/hessian
    info.energy_count(0);
    info.gradient_segment_count(0);
    info.hessian_block_count(0);
}

void AffineBodyExternalBodyForceConstraint::do_compute_energy(AffineBodyAnimator::EnergyInfo& info)
{
    // No energy computation
}

void AffineBodyExternalBodyForceConstraint::do_compute_gradient_hessian(
    AffineBodyAnimator::GradientHessianInfo& info)
{
    // No gradient/hessian computation
}
}  // namespace uipc::backend::cuda
