#include <sim_system_collection.h>

namespace uipc::backend::cuda
{
template <std::derived_from<SimSystem> T>
T* SimSystem::find()
{
    check_state(SimEngineState::BuildSystems, "find()");
    auto ptr = collection().find<T>();
    if(ptr)
    {
        SimSystem* p = ptr;
        if(!p->is_valid())
            this->set_invalid();
        m_dependencies.push_back(p);
    }
    return ptr;
}

template <std::derived_from<SimSystem> T>
T& uipc::backend::cuda::SimSystem::require()
{
    check_state(SimEngineState::BuildSystems, "require()");
    auto ptr = collection().find<T>();
    if(ptr)
    {
        SimSystem* p = ptr;
        if(!p->is_valid())
            this->set_invalid();
        m_dependencies.push_back(p);
    }
    else
    {
        throw SimSystemException(fmt::format("System {} not found", typeid(T).name()));
    }
    return *ptr;
}
}  // namespace uipc::backend::cuda
