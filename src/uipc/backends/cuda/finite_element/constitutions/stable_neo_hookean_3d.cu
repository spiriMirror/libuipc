#include <finite_element/constitutions/stable_neo_hookean_3d.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(StableNeoHookean3D);

U64 StableNeoHookean3D::get_constitution_uid() const
{
    return ConstitutionUID;
}

void StableNeoHookean3D::do_build(BuildInfo& info) {}
}  // namespace uipc::backend::cuda
