#pragma once
#include <uipc/core/affine_body_state_accessor_feature.h>

namespace uipc::backend::cuda
{
class AffineBodyDynamics;

class AffineBodyStateAccessorFeatureOverrider final : public core::AffineBodyStateAccessorFeatureOverrider
{
  public:
    AffineBodyStateAccessorFeatureOverrider(AffineBodyDynamics* abd);

    SizeT get_body_count() override;
    void  do_copy_from(const geometry::SimplicialComplex& state_geo) override;
    void  do_copy_to(geometry::SimplicialComplex& state_geo) override;

  private:
    AffineBodyDynamics*      m_abd;
    mutable vector<Vector12> m_buffer;
};
}  // namespace uipc::backend::cuda
