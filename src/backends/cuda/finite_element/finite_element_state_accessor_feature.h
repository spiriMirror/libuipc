#pragma once
#include <uipc/core/finite_element_state_accessor_feature.h>

namespace uipc::backend::cuda
{
class FiniteElementMethod;
class FiniteElementStateAccessorFeatureOverrider final : public core::FiniteElementStateAccessorFeatureOverrider
{
  public:
    FiniteElementStateAccessorFeatureOverrider(FiniteElementMethod* fem);

    SizeT get_vertex_count() override;
    void  do_copy_from(const geometry::SimplicialComplex& state_geo) override;
    void  do_copy_to(geometry::SimplicialComplex& state_geo) override;

  private:
    FiniteElementMethod* m_fem = nullptr;
};
}  // namespace uipc::backend::cuda
