#include <affine_body/affine_body_state_accessor_feature.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/utils.h>
#include <muda/launch/parallel_for.h>

namespace uipc::backend::cuda
{
AffineBodyStateAccessorFeatureOverrider::AffineBodyStateAccessorFeatureOverrider(
    AffineBodyDynamics& abd, AffineBodyVertexReporter& vertex_reporter)
    : m_abd{abd}
    , m_vertex_reporter{vertex_reporter}
{
}

SizeT AffineBodyStateAccessorFeatureOverrider::get_body_count()
{
    return m_abd.qs().size();
}

void AffineBodyStateAccessorFeatureOverrider::do_copy_from(const geometry::SimplicialComplex& state_geo)
{
    auto q_offset_attr = state_geo.meta().find<IndexT>(builtin::backend_abd_body_offset);
    UIPC_ASSERT(q_offset_attr, "Cannot find `backend_abd_body_offset` on State Geometry, why can it happen?");
    auto q_offset = q_offset_attr->view()[0];
    auto q_count  = state_geo.instances().size();

    // 1. Transform
    auto trans = state_geo.instances().find<Matrix4x4>(builtin::transform);
    m_buffer.resize(q_count);
    if(trans)
    {
        auto trans_view = trans->view();
        std::ranges::transform(trans_view, m_buffer.begin(), transform_to_q);
        auto q_subview = m_abd.m_impl.body_id_to_q.view(q_offset, q_count);
        q_subview.copy_from(m_buffer.data());
    }

    // 2. Velocity
    auto vel = state_geo.instances().find<Matrix4x4>(builtin::velocity);
    if(vel)
    {
        auto vel_view = vel->view();
        std::ranges::transform(vel_view, m_buffer.begin(), transform_v_to_q_v);
        auto q_v_subview = m_abd.m_impl.body_id_to_q_v.view(q_offset, q_count);
        q_v_subview.copy_from(m_buffer.data());
    }

    // request the vertex reporter to update attributes
    m_vertex_reporter.request_attribute_update();
}

void AffineBodyStateAccessorFeatureOverrider::do_copy_to(geometry::SimplicialComplex& state_geo)
{
    auto q_offset_attr = state_geo.meta().find<IndexT>(builtin::backend_abd_body_offset);
    UIPC_ASSERT(q_offset_attr, "Cannot find `backend_abd_body_offset` on State Geometry, why can it happen?");
    auto q_offset = q_offset_attr->view()[0];
    auto q_count  = state_geo.instances().size();

    // 1. Transform
    auto trans = state_geo.instances().find<Matrix4x4>(builtin::transform);
    m_buffer.resize(q_count);
    if(trans)
    {
        auto trans_view = view(*trans);
        auto q_subview  = m_abd.qs().subview(q_offset, q_count);
        q_subview.copy_to(m_buffer.data());
        std::ranges::transform(m_buffer, trans_view.begin(), q_to_transform);
    }

    // 2. Velocity
    auto vel = state_geo.instances().find<Matrix4x4>(builtin::velocity);
    if(vel)
    {
        auto vel_view    = view(*vel);
        auto q_v_subview = m_abd.q_vs().subview(q_offset, q_count);
        q_v_subview.copy_to(m_buffer.data());
        std::ranges::transform(m_buffer, vel_view.begin(), q_v_to_transform_v);
    }
}


void AffineBodyStateAccessorFeatureOverrider::do_copy_transform_to(
    backend::BufferView buffer_view, IndexT body_offset, SizeT body_count)
{
    auto q_view    = m_abd.qs();
    auto q_subview = q_view.subview(body_offset, body_count);

    auto* dst_ptr = reinterpret_cast<Matrix4x4*>(buffer_view.handle()) + buffer_view.offset();

    muda::ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(body_count,
               [q_in = q_subview.cviewer().name("q_in"),
                dst  = dst_ptr] __device__(int i) mutable
               { dst[i] = q_to_transform(q_in(i)); });
}

void AffineBodyStateAccessorFeatureOverrider::do_copy_velocity_to(
    backend::BufferView buffer_view, IndexT body_offset, SizeT body_count)
{
    auto q_v_view    = m_abd.q_vs();
    auto q_v_subview = q_v_view.subview(body_offset, body_count);

    auto* dst_ptr = reinterpret_cast<Matrix4x4*>(buffer_view.handle()) + buffer_view.offset();

    muda::ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(body_count,
               [q_in = q_v_subview.cviewer().name("q_v_in"),
                dst  = dst_ptr] __device__(int i) mutable
               { dst[i] = q_v_to_transform_v(q_in(i)); });
}
}  // namespace uipc::backend::cuda
