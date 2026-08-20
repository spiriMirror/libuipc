#pragma once
#include <type_define.h>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
class AffineBodyDynamics;
class AffineBodyVertexReporter;
class GlobalJointDofManager;
class AffineBodyStateAccessorFeatureOverrider final : public core::AffineBodyStateAccessorFeatureOverrider
{
  public:
    AffineBodyStateAccessorFeatureOverrider(AffineBodyDynamics& abd,
                                            AffineBodyVertexReporter& vertex_reporter,
                                            GlobalJointDofManager&    joint_dof_manager);

    SizeT get_body_count() override;
    void  do_copy_from(const geometry::SimplicialComplex& state_geo) override;
    void  do_copy_to(geometry::SimplicialComplex& state_geo) override;

    void do_copy_transform_to(backend::BufferView buffer_view, IndexT body_offset, SizeT body_count) override;
    void do_copy_velocity_to(backend::BufferView buffer_view, IndexT body_offset, SizeT body_count) override;

  private:
    AffineBodyDynamics&       m_abd;
    AffineBodyVertexReporter& m_vertex_reporter;
    GlobalJointDofManager&    m_joint_dof_manager;
    mutable vector<Vector12>  m_buffer;
};
}  // namespace uipc::backend::cuda
