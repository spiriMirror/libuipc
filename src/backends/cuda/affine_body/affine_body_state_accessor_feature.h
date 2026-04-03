#pragma once
#include <type_define.h>
#include <uipc/core/affine_body_state_accessor_feature.h>
#include <muda/buffer/device_buffer.h>

namespace uipc::backend::cuda
{
class AffineBodyDynamics;
class AffineBodyVertexReporter;
class AffineBodyStateAccessorFeatureOverrider final : public core::AffineBodyStateAccessorFeatureOverrider
{
  public:
    AffineBodyStateAccessorFeatureOverrider(AffineBodyDynamics& abd,
                                            AffineBodyVertexReporter& vertex_reporter);

    SizeT get_body_count() override;
    void  do_copy_from(const geometry::SimplicialComplex& state_geo) override;
    void  do_copy_to(geometry::SimplicialComplex& state_geo) override;

    backend::BufferView do_get_transform_buffer(IndexT body_offset, SizeT body_count) override;
    backend::BufferView do_get_velocity_buffer(IndexT body_offset, SizeT body_count) override;

  private:
    AffineBodyDynamics&       m_abd;
    AffineBodyVertexReporter& m_vertex_reporter;
    mutable vector<Vector12>  m_buffer;

    mutable muda::DeviceBuffer<Matrix4x4> m_transform_buffer;
    mutable muda::DeviceBuffer<Matrix4x4> m_velocity_buffer;
};
}  // namespace uipc::backend::cuda
