#include <sim_system_collection.h>

namespace uipc::backend::cuda
{
template <std::derived_from<SimSystem> T>
T* SimSystem::find()
{
    return collection().find<T>();
}
}  // namespace uipc::backend::cuda
