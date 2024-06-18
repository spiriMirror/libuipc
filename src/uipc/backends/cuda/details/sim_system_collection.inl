#include <uipc/common/log.h>
#include <sim_system.h>
namespace uipc::backend::cuda
{
template <std::derived_from<ISimSystem> T>
T* SimSystemCollection::find()
{
    auto tid = typeid(T).hash_code();
    auto it  = m_sim_systems.find(tid);

    // find exact match
    if(it != m_sim_systems.end())
        return dynamic_cast<T*>(it->second.get());

    // if not found, find compatible match
    for(auto& [key, value] : m_sim_systems)
    {
        if(auto* ptr = dynamic_cast<T*>(value.get()); ptr != nullptr)
        {
            UIPC_WARN_WITH_LOCATION(
                "Unable to find the excat SimSystem <{}>, but found a compatible one <{}>, "
                "consider optimizing your implementation.",
                typeid(T).name(),
                typeid(*ptr).name());
            return ptr;
        }
    }

    // failed to find any match
    return nullptr;
}
}  // namespace uipc::backend::cuda