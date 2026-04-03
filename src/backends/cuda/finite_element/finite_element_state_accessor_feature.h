#pragma once
#include <type_define.h>
#include <uipc/core/finite_element_state_accessor_feature.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;
class FiniteElementVertexReporter;

class FiniteElementStateAccessorFeatureOverrider final : public core::FiniteElementStateAccessorFeatureOverrider
{
  public:
    FiniteElementStateAccessorFeatureOverrider(FiniteElementMethod& fem,
                                               FiniteElementVertexReporter& vertex_reporter);

    SizeT get_vertex_count() override;
    void  do_copy_from(const geometry::SimplicialComplex& state_geo) override;
    void  do_copy_to(geometry::SimplicialComplex& state_geo) override;

    backend::BufferView do_get_position_buffer(IndexT vertex_offset, SizeT vertex_count) override;
    backend::BufferView do_get_velocity_buffer(IndexT vertex_offset, SizeT vertex_count) override;

  private:
    FiniteElementMethod&         m_fem;
    FiniteElementVertexReporter& m_vertex_reporter;
};
}  // namespace uipc::backend::cuda
