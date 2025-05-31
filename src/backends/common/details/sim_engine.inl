#include <uipc/common/demangle.h>

namespace uipc::backend
{
template <std::derived_from<ISimSystem> T>
T* SimEngine::find()
{
    auto ptr = m_system_collection.find<T>();
    ptr      = static_cast<T*>(find_system(ptr));
    return ptr;
}

template <std::derived_from<ISimSystem> T>
T& SimEngine::require()
{
    auto ptr = m_system_collection.find<T>();
    ptr      = static_cast<T*>(require_system(ptr));
    if(!ptr)
    {
        throw SimEngineException(
            fmt::format("SimSystem [{}] not found", uipc::demangle<T>()));
    }
    return *ptr;
}
}  // namespace uipc::backend
