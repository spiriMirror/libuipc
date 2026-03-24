#include <affine_body/affine_body_state_accessor_feature.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/utils.h>

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

    // 3. Total mass
    auto total_mass_attr = state_geo.instances().find<Float>(builtin::total_mass);
    if(total_mass_attr)
    {
        vector<Float> tmp(q_count);
        auto src = m_abd.m_impl.body_id_to_total_mass.view(q_offset, q_count);
        src.copy_to(tmp.data());
        auto dst = view(*total_mass_attr);
        std::ranges::copy(tmp, dst.begin());
    }

    // 4. Inertia tensor
    auto inertia_tensor_attr =
        state_geo.instances().find<Matrix3x3>(builtin::inertia_tensor);
    if(inertia_tensor_attr)
    {
        vector<Matrix3x3> tmp(q_count);
        auto src = m_abd.m_impl.body_id_to_inertia_tensor.view(q_offset, q_count);
        src.copy_to(tmp.data());
        auto dst = view(*inertia_tensor_attr);
        std::ranges::copy(tmp, dst.begin());
    }
}
}  // namespace uipc::backend::cuda
