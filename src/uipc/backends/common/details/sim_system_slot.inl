namespace uipc::backend
{
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
SimSystemSlot<T>::SimSystemSlot(T& subsystem) noexcept
    : m_subsystem(&subsystem)
{
}

template <typename T>
T* const SimSystemSlot<T>::view() const noexcept
{
    lazy_init();
    return m_subsystem;
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

template <typename T>
void SimSystemSlot<T>::lazy_init() const
{
    if constexpr(uipc::RUNTIME_CHECK)
    {
        if(m_subsystem)
        {
            UIPC_ASSERT(!m_subsystem->is_building(),
                        "SimSystemSlot<{}>::lazy_init() should be called after building",
                        typeid(T).name());
        }
    }

    if(m_subsystem)
        m_subsystem = m_subsystem->is_valid() ? m_subsystem : nullptr;

    built = true;
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
                        typeid(T).name());
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