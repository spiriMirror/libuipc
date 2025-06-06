#include <uipc/common/demangle.h>
#include "sim_system_slot.h"

namespace uipc::backend
{
template <typename T>
void SimSystemSlot<T>::register_subsystem(T& subsystem)
{
    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from Subsystem");

    if(subsystem.is_valid())
        m_subsystem = &subsystem;
}

template <typename T>
SimSystemSlot<T>& SimSystemSlot<T>::operator=(T& subsystem) noexcept
{
    register_subsystem(subsystem);
    return *this;
}

template <typename T>
SimSystemSlot<T>& SimSystemSlot<T>::operator=(T* subsystem) noexcept
{
    if(subsystem)
    {
        register_subsystem(*subsystem);
    }
    else
    {
        m_subsystem = nullptr;  // reset to nullptr if subsystem is null
    }
    return *this;
}

template <typename T>
SimSystemSlot<T>::SimSystemSlot(T& subsystem) noexcept
    : m_subsystem(&subsystem)
{
}

template <typename T>
T* SimSystemSlot<T>::view() const noexcept
{
    lazy_init();
    return m_subsystem;
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
    if(m_subsystem)
    {
        // if the subsystem is not valid, set it to nullptr
        m_subsystem = m_subsystem->is_valid() ? m_subsystem : nullptr;
    }
}

template <typename T>
void SimSystemSlotCollection<T>::register_subsystem(T& subsystem)
{
    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from Subsystem");

    built = false;
    if(subsystem.is_valid())
        m_subsystem_buffer.push_back(&subsystem);
}

template <typename T>
span<T* const> SimSystemSlotCollection<T>::view() const noexcept
{
    lazy_init();
    return m_subsystems;
}

template <typename T>
span<T*> SimSystemSlotCollection<T>::view() noexcept
{
    lazy_init();
    return m_subsystems;
}


template <typename T>
void SimSystemSlotCollection<T>::lazy_init() const noexcept
{
    // accessing of the subsystems is only allowed after all `do_build()` called;
    if(!built)
    {
        static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from Subsystem");
        std::erase_if(m_subsystem_buffer,
                      [](T* subsystem) { return !subsystem->is_valid(); });

        if constexpr(uipc::RUNTIME_CHECK)
        {
            bool not_building =
                std::ranges::all_of(m_subsystem_buffer,
                                    [](T* subsystem)
                                    { return !subsystem->is_building(); });

            UIPC_ASSERT(not_building,
                        "SimSystemSlotCollection<{}>::lazy_init() should be called after building",
                        uipc::demangle<T>());
        }

        m_subsystems.reserve(m_subsystem_buffer.size());
        std::ranges::move(m_subsystem_buffer, std::back_inserter(m_subsystems));
        m_subsystem_buffer.clear();
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
        for(T* subsystem : m_subsystem_buffer)
        {

            UIPC_ASSERT(!subsystem->is_building(),
                        "`find()` can't be called while subsystems are being built");

            if(auto casted = dynamic_cast<U*>(subsystem))
            {
                return SimSystemSlot<U>{*casted};
            }
        }
    }
    else
    {
        for(T* subsystem : m_subsystems)
        {
            UIPC_ASSERT(!subsystem->is_building(),
                        "`find()` can't be called while subsystems are being built");

            if(auto casted = dynamic_cast<U*>(subsystem))
            {
                return SimSystemSlot<U>{*casted};
            }
        }
    }

    return SimSystemSlot<U>{};
}
}  // namespace uipc::backend