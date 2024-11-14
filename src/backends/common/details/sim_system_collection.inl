#include <uipc/common/log.h>
#include <backends/common/i_sim_system.h>
namespace uipc::backend
{
template <std::derived_from<ISimSystem> T>
T* SimSystemCollection::find(const QueryOptions& options)
{
    auto tid = typeid(T).hash_code();
    auto it  = m_sim_system_map.find(tid);

    // look for exact match
    if(it != m_sim_system_map.end())
        return dynamic_cast<T*>(it->second.get());

    if(!options.exact)  // if allow compatible match
    {
        // if not found, look for compatible match
        for(auto& [key, value] : m_sim_system_map)
        {
            if(auto* ptr = dynamic_cast<T*>(value.get()); ptr != nullptr)
            {
                return ptr;
            }
        }
    }

    // failed to find any match
    return nullptr;
}
}  // namespace uipc::backend