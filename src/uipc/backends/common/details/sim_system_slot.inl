namespace uipc::backend
{
template <typename T>
void SimSystemSlotCollection<T>::register_subsystem(T& subsystem)
{

    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from Subsystem");
    UIPC_ASSERT(subsystem.is_building(),
                "`register_subsystem()` can only be called when the SimEngine builds system.");

    built = false;
    if(subsystem.is_valid())
        m_subsystem_buffer.push_back(&subsystem);
}

template <typename T>
void SimSystemSlotCollection<T>::init()
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
                    "SimSystemSlotCollection<{}>::init() should be called after building",
                    typeid(T).name());
    }

    m_subsystems.reserve(m_subsystem_buffer.size());
    std::ranges::move(m_subsystem_buffer, std::back_inserter(m_subsystems));
    m_subsystem_buffer.clear();
    built = true;
}

template <typename T>
span<T*> SimSystemSlotCollection<T>::view() noexcept
{
    check_build();
    return m_subsystems;
}

template <typename T>
span<T* const> SimSystemSlotCollection<T>::view() const noexcept
{
    check_build();
    return m_subsystems;
}

template <typename T>
inline void SimSystemSlotCollection<T>::check_build() const noexcept
{
    UIPC_ASSERT(built,
                "SimSystemSlotCollection<{}> is not built yet. Call init() before accessing the subsystems",
                typeid(T).name());
}

template <typename T>
void SimSystemSlot<T>::register_subsystem(T& subsystem)
{
    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from Subsystem");

    UIPC_ASSERT(subsystem.is_building(),
                "`register_subsystem()` can only be called when the SimEngine builds system.");
    UIPC_ASSERT(!m_subsystem,
                "SimSystemSlot[{}] already registered, yours [{}]",
                m_subsystem->name(),
                subsystem.name());

    built = false;
    if(subsystem.is_valid())
        m_subsystem = &subsystem;
}

template <typename T>
void SimSystemSlot<T>::init()
{

    static_assert(std::is_base_of_v<ISimSystem, T>, "T must be derived from Subsystem");

    if constexpr(uipc::RUNTIME_CHECK)
    {
        if(m_subsystem)
        {
            UIPC_ASSERT(!m_subsystem->is_building(),
                        "SimSystemSlot<{}>::init() should be called after building",
                        typeid(T).name());
        }
    }

    if(m_subsystem)
        m_subsystem = m_subsystem->is_valid() ? m_subsystem : nullptr;

    built = true;
}

template <typename T>
inline void SimSystemSlot<T>::check_build() const noexcept
{
    UIPC_ASSERT(built,
                "SimSystemSlot<{}> is not built yet. Call init() before accessing the subsystems",
                typeid(T).name());
}

template <typename T>
inline T* SimSystemSlot<T>::view() noexcept
{
    check_build();
    return m_subsystem;
}

template <typename T>
inline T* const SimSystemSlot<T>::view() const noexcept
{
    check_build();
    return m_subsystem;
}

template <typename T>
T* SimSystemSlot<T>::operator->() noexcept
{
    return view();
}

template <typename T>
T* const SimSystemSlot<T>::operator->() const noexcept
{
    return view();
}

template <typename T>
SimSystemSlot<T>::operator bool() const noexcept
{
    return view();
}
}  // namespace uipc::backend