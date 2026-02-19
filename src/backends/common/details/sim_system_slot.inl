#include <uipc/common/demangle.h>

namespace uipc::backend
{
template <typename T>
void SimSystemSlot<T>::register_sim_system(T& sim_system)
{
    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from sim_system");

    if(sim_system.is_valid())
        m_sim_system = &sim_system;
}

template <typename T>
SimSystemSlot<T>& SimSystemSlot<T>::operator=(T& sim_system) noexcept
{
    register_sim_system(sim_system);
    return *this;
}

template <typename T>
SimSystemSlot<T>& SimSystemSlot<T>::operator=(T* sim_system) noexcept
{
    if(sim_system)
    {
        register_sim_system(*sim_system);
    }
    else
    {
        m_sim_system = nullptr;  // reset to nullptr if sim_system is null
    }
    return *this;
}

template <typename T>
SimSystemSlot<T>::SimSystemSlot(T& sim_system) noexcept
    : m_sim_system(&sim_system)
{
}

template <typename T>
SimSystemSlot<T>::SimSystemSlot(T* sim_system) noexcept
    : m_sim_system(sim_system)
{
}

template <typename T>
T* SimSystemSlot<T>::view() const noexcept
{
    lazy_init();
    return m_sim_system;
}

template <typename T>
T* SimSystemSlot<T>::operator->() const noexcept
{
    return view();
}

template <typename T>
SimSystemSlot<T>::operator bool() const noexcept
{
    return view();
}

template <typename T>
void SimSystemSlot<T>::lazy_init() const
{
    if(m_sim_system)
    {
        // if the sim_system is not valid, set it to nullptr
        m_sim_system = m_sim_system->is_valid() ? m_sim_system : nullptr;
    }
}

template <typename T>
void SimSystemSlotCollection<T>::register_sim_system(T& sim_system)
{
    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from sim_system");

    built = false;
    if(sim_system.is_valid())
        m_sim_system_buffer.push_back(&sim_system);
}

template <typename T>
span<T* const> SimSystemSlotCollection<T>::view() const noexcept
{
    lazy_init();
    return m_sim_systems;
}

template <typename T>
span<T*> SimSystemSlotCollection<T>::view() noexcept
{
    lazy_init();
    return m_sim_systems;
}


template <typename T>
void SimSystemSlotCollection<T>::lazy_init() const noexcept
{
    // accessing of the sim_systems is only allowed after all `do_build()` called;
    if(!built)
    {
        static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from sim_system");
        std::erase_if(m_sim_system_buffer,
                      [](T* sim_system) { return !sim_system->is_valid(); });

        if constexpr(uipc::RUNTIME_CHECK)
        {
            bool not_building =
                std::ranges::all_of(m_sim_system_buffer,
                                    [](T* sim_system)
                                    { return !sim_system->is_building(); });

            UIPC_ASSERT(not_building,
                        "SimSystemSlotCollection<{}>::lazy_init() should be called after building",
                        uipc::demangle<T>());
        }

        m_sim_systems.reserve(m_sim_system_buffer.size());
        std::ranges::move(m_sim_system_buffer, std::back_inserter(m_sim_systems));
        m_sim_system_buffer.clear();
        built = true;
    }
}

template <typename T>
template <typename U>
SimSystemSlot<U> SimSystemSlotCollection<T>::find() const noexcept
{
    static_assert(std::is_base_of_v<T, U>, "U must be derived from T");

    if(!built)
    {
        for(T* sim_system : m_sim_system_buffer)
        {

            UIPC_ASSERT(!sim_system->is_building(),
                        "`find()` can't be called while sim_systems are being built");

            if(auto casted = dynamic_cast<U*>(sim_system))
            {
                return SimSystemSlot<U>{*casted};
            }
        }
    }
    else
    {
        for(T* sim_system : m_sim_systems)
        {
            UIPC_ASSERT(!sim_system->is_building(),
                        "`find()` can't be called while sim_systems are being built");

            if(auto casted = dynamic_cast<U*>(sim_system))
            {
                return SimSystemSlot<U>{*casted};
            }
        }
    }

    return SimSystemSlot<U>{};
}
}  // namespace uipc::backend