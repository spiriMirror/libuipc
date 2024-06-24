#include <sim_system_collection.h>

namespace uipc::backend::cuda
{
template <std::derived_from<SimSystem> T>
T* SimSystem::find()
{
    auto ptr = collection().find<T>();
    if(ptr)
        m_dependencies.push_back(ptr);
    return ptr;
}
}  // namespace uipc::backend::cuda
