#include <affine_body/affine_body_state_accessor_feature.h>
#include <affine_body/affine_body_dynamics.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <joint_dof_system/global_joint_dof_manager.h>
#include <uipc/builtin/attribute_name.h>
#include <affine_body/utils.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void affine_body_state_accessor_feature_copy_transform_to_kernel(
        cuda_tool::CBufferView<Vector12> q_in, cuda_tool::BufferView<Matrix4x4> dst, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        dst(i) = q_to_transform(q_in(i));
    }

    __global__ void affine_body_state_accessor_feature_copy_velocity_to_kernel(
        cuda_tool::CBufferView<Vector12> q_in, cuda_tool::BufferView<Matrix4x4> dst, int n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        dst(i) = q_v_to_transform_v(q_in(i));
    }
}  // namespace

AffineBodyStateAccessorFeatureOverrider::AffineBodyStateAccessorFeatureOverrider(
    AffineBodyDynamics& abd, AffineBodyVertexReporter& vertex_reporter, GlobalJointDofManager& joint_dof_manager)
    : m_abd{abd}
    , m_vertex_reporter{vertex_reporter}
    , m_joint_dof_manager{joint_dof_manager}
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
    // re-sync persistent per-joint DOF state (e.g. revolute current_angles)
    // synchronously so it stays consistent with the freshly written body q.
    m_joint_dof_manager.update_dof_attributes();
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

    auto* dst_ptr =
        reinterpret_cast<Matrix4x4*>(buffer_view.handle()) + buffer_view.offset();
    cuda_tool::BufferView<Matrix4x4> dst_view{dst_ptr, body_count};

    int n = (int)body_count;
    if(n > 0)
    {
        auto k = affine_body_state_accessor_feature_copy_transform_to_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            q_subview.cview(), dst_view, n);
    }
}

void AffineBodyStateAccessorFeatureOverrider::do_copy_velocity_to(backend::BufferView buffer_view,
                                                                  IndexT body_offset,
                                                                  SizeT body_count)
{
    auto q_v_view    = m_abd.q_vs();
    auto q_v_subview = q_v_view.subview(body_offset, body_count);

    auto* dst_ptr =
        reinterpret_cast<Matrix4x4*>(buffer_view.handle()) + buffer_view.offset();
    cuda_tool::BufferView<Matrix4x4> dst_view{dst_ptr, body_count};

    int n = (int)body_count;
    if(n > 0)
    {
        auto k = affine_body_state_accessor_feature_copy_velocity_to_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            q_v_subview.cview(), dst_view, n);
    }
}
}  // namespace uipc::backend::cuda
