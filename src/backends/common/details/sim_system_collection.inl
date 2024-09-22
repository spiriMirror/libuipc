#include <uipc/common/log.h>
#include <backends/common/i_sim_system.h>
namespace uipc::backend
{
template <std::derived_from<ISimSystem> T>
T* SimSystemCollection::find()
{
    auto tid = typeid(T).hash_code();
    auto it  = m_sim_system_map.find(tid);

    // find exact match
    if(it != m_sim_system_map.end())
        return dynamic_cast<T*>(it->second.get());

    // if not found, find compatible match
    for(auto& [key, value] : m_sim_system_map)
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
}  // namespace uipc::backend