#include <backends/common/sim_system_collection.h>

namespace uipc::backend
{
template <std::derived_from<SimSystem> T>
T* SimSystem::find()
{
    check_state("find()");
    auto ptr = collection().find<T>();
    ptr      = static_cast<T*>(find_system(ptr));
    return ptr;
}

template <std::derived_from<SimSystem> T>
T& SimSystem::require()
{
    check_state("require()");
    auto ptr = collection().find<T>();
    ptr      = static_cast<T*>(require_system(ptr));
    if(!ptr)
    {
        throw SimSystemException(fmt::format("SimSystem [{}] not found", typeid(T).name()));
    }
    return *ptr;
}
}  // namespace uipc::backend
