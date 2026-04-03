#include <finite_element/finite_element_state_accessor_feature.h>
#include <finite_element/finite_element_method.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{

backend::BufferView copy_and_get_buffer(muda::CBufferView<Vector3>   src_view,
                                        muda::DeviceBuffer<Vector3>& out_buffer,
                                        IndexT vertex_offset,
                                        SizeT  vertex_count)
{
    auto total = src_view.size();

    // resize output buffer if needed
    if(out_buffer.size() != total)
        out_buffer.resize(total);

    // copy the requested range to the read-only buffer
    auto src_subview = src_view.subview(vertex_offset, vertex_count);
    auto dst_subview = out_buffer.view(vertex_offset, vertex_count);
    dst_subview.copy_from(src_subview);

    auto* ptr = out_buffer.data() + vertex_offset;
    return backend::BufferView{
        reinterpret_cast<HandleT>(ptr), 0, vertex_count, sizeof(Vector3), sizeof(Vector3), "cuda"};
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

backend::BufferView FiniteElementStateAccessorFeatureOverrider::do_get_position_buffer(
    IndexT vertex_offset, SizeT vertex_count)
{
    return copy_and_get_buffer(m_fem.xs(), m_position_buffer, vertex_offset, vertex_count);
}

backend::BufferView FiniteElementStateAccessorFeatureOverrider::do_get_velocity_buffer(
    IndexT vertex_offset, SizeT vertex_count)
{
    return copy_and_get_buffer(m_fem.vs(), m_velocity_buffer, vertex_offset, vertex_count);
}
}  // namespace uipc::backend::cuda
