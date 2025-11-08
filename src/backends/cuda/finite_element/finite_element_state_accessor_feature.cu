#include <finite_element/finite_element_state_accessor_feature.h>
#include <finite_element/finite_element_method.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
FiniteElementStateAccessorFeatureOverrider::FiniteElementStateAccessorFeatureOverrider(FiniteElementMethod* fem)
    : m_fem{fem}
{
    UIPC_ASSERT(fem, "FiniteElementMethod cannot be null");
}

SizeT FiniteElementStateAccessorFeatureOverrider::get_vertex_count()
{
    return m_fem->xs().size();
}

void FiniteElementStateAccessorFeatureOverrider::do_copy_from(const geometry::SimplicialComplex& state_geo)
{
    auto v_offset_attr = state_geo.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
    UIPC_ASSERT(v_offset_attr, "Cannot find `backend_fem_vertex_offset` on State Geometry, why can it happen?");
    auto v_offset = v_offset_attr->view()[0];
    auto v_count  = state_geo.vertices().size();


    // 1. Position
    auto pos = state_geo.vertices().find<Vector3>(builtin::position);
    if(pos)
    {
        auto pos_view  = pos->view();
        auto x_subview = m_fem->m_impl.xs.view(v_offset, v_count);
        x_subview.copy_from(pos_view.data());
    }

    // 2. Velocity
    auto vel = state_geo.vertices().find<Vector3>(builtin::velocity);
    if(vel)
    {
        auto vel_view  = vel->view();
        auto v_subview = m_fem->m_impl.vs.view(v_offset, v_count);
        v_subview.copy_from(vel_view.data());
    }
}

void FiniteElementStateAccessorFeatureOverrider::do_copy_to(geometry::SimplicialComplex& state_geo)
{
    auto v_offset_attr = state_geo.meta().find<IndexT>(builtin::backend_fem_vertex_offset);
    UIPC_ASSERT(v_offset_attr, "Cannot find `backend_fem_vertex_offset` on State Geometry, why can it happen?");
    auto v_offset = v_offset_attr->view()[0];
    auto v_count  = state_geo.vertices().size();

    // 1. Position
    auto pos = state_geo.vertices().find<Vector3>(builtin::position);
    if(pos)
    {
        auto pos_view  = view(*pos);
        auto x_subview = m_fem->m_impl.xs.view(v_offset, v_count);
        x_subview.copy_to(pos_view.data());
    }

    // 2. Velocity
    auto vel = state_geo.vertices().find<Vector3>(builtin::velocity);
    if(vel)
    {
        auto vel_view  = view(*vel);
        auto v_subview = m_fem->m_impl.vs.view(v_offset, v_count);
        v_subview.copy_to(vel_view.data());
    }
}
}  // namespace uipc::backend::cuda
