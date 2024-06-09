#include <uipc/backends/cuda/affine_body_geometry.h>
#include <uipc/backends/cuda/sim_system_auto_register.h>

namespace uipc::backend::cuda
{
void AffineBodyGeometry::build() noexcept
{
    auto& world = this->engine().world();
    auto  s     = world.scene();
}


REGISTER_SIM_SYSTEM(AffineBodyGeometry);
}  // namespace uipc::backend::cuda
