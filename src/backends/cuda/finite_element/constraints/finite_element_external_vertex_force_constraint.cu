#include <finite_element/constraints/finite_element_external_vertex_force_constraint.h>
#include <finite_element/finite_element_method.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FiniteElementExternalVertexForceConstraint);

void FiniteElementExternalVertexForceConstraint::do_build(BuildInfo& info) {}

U64 FiniteElementExternalVertexForceConstraint::get_uid() const noexcept
{
    return UID;
}

void FiniteElementExternalVertexForceConstraint::do_init(FiniteElementAnimator::FilteredInfo& info)
{
    do_step(info);
}

muda::CBufferView<Vector3> FiniteElementExternalVertexForceConstraint::forces() const noexcept
{
    return m_impl.forces.view();
}

muda::CBufferView<IndexT> FiniteElementExternalVertexForceConstraint::vertex_ids() const noexcept
{
    return m_impl.vertex_ids.view();
}

void FiniteElementExternalVertexForceConstraint::Impl::step(
    backend::WorldVisitor&              world,
    FiniteElementAnimator::FilteredInfo& info)
{
    using ForEachInfo = FiniteElementMethod::ForEachInfo;

    h_forces.clear();
    h_vertex_ids.clear();

    auto   geo_slots             = world.scene().geometries();
    IndexT current_vertex_offset = 0;

    info.for_each(
        geo_slots,
        [&](geometry::SimplicialComplex& sc)
        {
            auto vertex_offset =
                sc.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
            UIPC_ASSERT(vertex_offset,
                        "`backend_fem_vertex_offset` attribute not found");
            current_vertex_offset = vertex_offset->view().front();

            auto is_constrained = sc.vertices().find<IndexT>(builtin::is_constrained);
            UIPC_ASSERT(is_constrained,
                        "`is_constrained` attribute not found");
            auto external_force = sc.vertices().find<Vector3>("external_force");
            UIPC_ASSERT(external_force,
                        "`external_force` attribute not found");

            return zip(is_constrained->view(), external_force->view());
        },
        [&](const ForEachInfo& I, auto&& values)
        {
            auto vI = I.local_index() + current_vertex_offset;
            auto&& [is_constrained, force] = values;
            if(is_constrained)
            {
                h_vertex_ids.push_back(vI);
                h_forces.push_back(force);
            }
        });

    if(!h_forces.empty())
    {
        forces.copy_from(h_forces);
        vertex_ids.copy_from(h_vertex_ids);
    }
}

void FiniteElementExternalVertexForceConstraint::do_step(FiniteElementAnimator::FilteredInfo& info)
{
    m_impl.step(world(), info);
}

void FiniteElementExternalVertexForceConstraint::do_report_extent(
    FiniteElementAnimator::ReportExtentInfo& info)
{
    info.energy_count(0);
    info.gradient_count(0);
    info.hessian_count(0);
}

void FiniteElementExternalVertexForceConstraint::do_compute_energy(
    FiniteElementAnimator::ComputeEnergyInfo& info)
{
}

void FiniteElementExternalVertexForceConstraint::do_compute_gradient_hessian(
    FiniteElementAnimator::ComputeGradientHessianInfo& info)
{
}
}  // namespace uipc::backend::cuda
