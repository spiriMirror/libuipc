#include <muda/ext/eigen/eigen_cxx20.h>
#include <affine_body_geometry.h>

namespace uipc::backend::cuda
{
void AffineBodyGeometry::build()
{
    // Register the action to initialize the affine body geometry
    on_init_scene([this] { init_affine_body_geometry(); });
}

void AffineBodyGeometry::init_affine_body_geometry() 
{
    auto& world = this->world();
}
}  // namespace uipc::backend::cuda

// Register the system
REGISTER_SIM_SYSTEM(uipc::backend::cuda::AffineBodyGeometry);