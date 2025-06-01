#include <backends/common/sim_system_collection.h>
#include <uipc/common/demangle.h>

namespace uipc::backend
{
template <std::derived_from<SimSystem> T>
T* SimSystem::find(const QueryOptions& options)
{
    check_state("find()");
    auto ptr = collection().find<T>(options);
    ptr      = static_cast<T*>(find_system(ptr));
    return ptr;
}

template <std::derived_from<SimSystem> T>
T& SimSystem::require(const QueryOptions& options)
{
    check_state("require()");
    auto ptr = collection().find<T>(options);
    ptr      = static_cast<T*>(require_system(ptr));
    if(!ptr)
    {
        throw SimSystemException(
            fmt::format("SimSystem [{}] not found", uipc::demangle<T>()));
    }
    return *ptr;
}
}  // namespace uipc::backend
