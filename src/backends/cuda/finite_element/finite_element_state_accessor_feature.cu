#include <finite_element/finite_element_state_accessor_feature.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{

void copy_to_buffer_view(muda::CBufferView<Vector3>   src_view,
                         backend::BufferView          buffer_view,
                         IndexT                       vertex_offset,
                         SizeT                        vertex_count)
{
    auto src_subview = src_view.subview(vertex_offset, vertex_count);
    auto* dst_ptr = reinterpret_cast<Vector3*>(buffer_view.handle()) + buffer_view.offset();
    muda::BufferView<Vector3> dst_muda_view{dst_ptr, vertex_count};
    dst_muda_view.copy_from(src_subview);
}

FiniteElementStateAccessorFeatureOverrider::FiniteElementStateAccessorFeatureOverrider(
    FiniteElementMethod& fem, FiniteElementVertexReporter& vertex_reporter)
    : m_fem{fem}
    , m_vertex_reporter{vertex_reporter}
{
}

SizeT FiniteElementStateAccessorFeatureOverrider::get_vertex_count()
{
    return m_fem.xs().size();
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
        auto x_subview = m_fem.m_impl.xs.view(v_offset, v_count);
        x_subview.copy_from(pos_view.data());
    }

    // 2. Velocity
    auto vel = state_geo.vertices().find<Vector3>(builtin::velocity);
    if(vel)
    {
        auto vel_view  = vel->view();
        auto v_subview = m_fem.m_impl.vs.view(v_offset, v_count);
        v_subview.copy_from(vel_view.data());
    }

    // request the vertex reporter to update attributes
    m_vertex_reporter.request_attribute_update();
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
        auto x_subview = m_fem.m_impl.xs.view(v_offset, v_count);
        x_subview.copy_to(pos_view.data());
    }

    // 2. Velocity
    auto vel = state_geo.vertices().find<Vector3>(builtin::velocity);
    if(vel)
    {
        auto vel_view  = view(*vel);
        auto v_subview = m_fem.m_impl.vs.view(v_offset, v_count);
        v_subview.copy_to(vel_view.data());
    }
}

void FiniteElementStateAccessorFeatureOverrider::do_copy_position_to(
    backend::BufferView buffer_view, IndexT vertex_offset, SizeT vertex_count)
{
    copy_to_buffer_view(m_fem.xs(), buffer_view, vertex_offset, vertex_count);
}

void FiniteElementStateAccessorFeatureOverrider::do_copy_velocity_to(
    backend::BufferView buffer_view, IndexT vertex_offset, SizeT vertex_count)
{
    copy_to_buffer_view(m_fem.vs(), buffer_view, vertex_offset, vertex_count);
}
}  // namespace uipc::backend::cuda
