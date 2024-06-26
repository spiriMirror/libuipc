#include <sim_system_creator_utils.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
bool has_affine_body_constitution(SimEngine& engine) noexcept
{
    const auto& scene = engine.world().scene();
    auto&       types = scene.constitution_tabular().types();
    return types.find(world::ConstitutionTypes::AffineBody) != types.end();
}
}  // namespace uipc::backend::cuda
