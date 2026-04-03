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

static backend::BufferView convert_and_get_buffer(muda::CBufferView<Vector12>    q_view,
                                                   muda::DeviceBuffer<Matrix4x4>& out_buffer,
                                                   IndexT body_offset,
                                                   SizeT  body_count,
                                                   bool   is_velocity)
{
    auto total = q_view.size();

    // resize output buffer to total body count if needed
    if(out_buffer.size() != total)
        out_buffer.resize(total);

    // convert Vector12 -> Matrix4x4 on GPU
    auto q_subview   = q_view.subview(body_offset, body_count);
    auto out_subview = out_buffer.view(body_offset, body_count);

    muda::ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(body_count,
               [is_velocity,
                q_in  = q_subview.cviewer().name("q_in"),
                m_out = out_subview.viewer().name("m_out")] __device__(int i) mutable
               {
                   if(is_velocity)
                       m_out(i) = q_v_to_transform_v(q_in(i));
                   else
                       m_out(i) = q_to_transform(q_in(i));
               });

    // construct BufferView from device pointer
    auto* ptr = out_buffer.data() + body_offset;
    return backend::BufferView{reinterpret_cast<HandleT>(ptr),
                               0,
                               body_count,
                               sizeof(Matrix4x4),
                               sizeof(Matrix4x4),
                               "cuda"};
}

backend::BufferView AffineBodyStateAccessorFeatureOverrider::do_get_transform_buffer(
    IndexT body_offset, SizeT body_count)
{
    return convert_and_get_buffer(
        m_abd.qs(), m_transform_buffer, body_offset, body_count, false);
}

backend::BufferView AffineBodyStateAccessorFeatureOverrider::do_get_velocity_buffer(
    IndexT body_offset, SizeT body_count)
{
    return convert_and_get_buffer(
        m_abd.q_vs(), m_velocity_buffer, body_offset, body_count, true);
}
}  // namespace uipc::backend::cuda
