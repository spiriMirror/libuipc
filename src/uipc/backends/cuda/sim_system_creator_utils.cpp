#include <sim_system_creator_utils.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
bool CreatorQuery::has_affine_body_constitution(SimEngine& engine) noexcept
{
    const auto& scene = engine.world().scene();
    auto&       types = scene.constitution_tabular().types();
    return types.find(world::ConstitutionTypes::AffineBody) != types.end();
}

bool CreatorQuery::is_contact_enabled(SimEngine& engine) noexcept
{
    auto& info = engine.world().scene().info();
    return info["contact"]["enable"].get<bool>();
}
}  // namespace uipc::backend::cuda
