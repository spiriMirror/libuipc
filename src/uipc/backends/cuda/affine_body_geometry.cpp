#include <affine_body_geometry.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
void AffineBodyGeometry::build() noexcept
{
}

REGISTER_SIM_SYSTEM(AffineBodyGeometry);
}  // namespace uipc::backend::cuda
