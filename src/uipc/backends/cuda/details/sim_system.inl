#include <sim_system_collection.h>

namespace uipc::backend::cuda
{
template <std::derived_from<SimSystem> T>
T* SimSystem::find()
{
    check_state(SimEngineState::BuildSystems, "find()");
    auto ptr = collection().find<T>();
    ptr      = static_cast<T*>(find_system(ptr));
    return ptr;
}

template <std::derived_from<SimSystem> T>
T& uipc::backend::cuda::SimSystem::require()
{
    check_state(SimEngineState::BuildSystems, "require()");
    auto ptr = collection().find<T>();
    ptr      = static_cast<T*>(require_system(ptr));
    if(!ptr)
    {
        throw SimSystemException(fmt::format("System {} not found", typeid(T).name()));
    }
    return *ptr;
}
}  // namespace uipc::backend::cuda
