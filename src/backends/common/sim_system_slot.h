#pragma once
#include <backends/common/i_sim_system.h>

namespace uipc::backend
{
template <typename T>
class SimSystemSlotCollection;

template <typename T>
class SimSystemSlot
{
  public:
    // default constructor
    SimSystemSlot() noexcept = default;
    // default copy/move
    SimSystemSlot(const SimSystemSlot&) noexcept            = default;
    SimSystemSlot(SimSystemSlot&&) noexcept                 = default;
    SimSystemSlot& operator=(const SimSystemSlot&) noexcept = default;
    SimSystemSlot& operator=(SimSystemSlot&&) noexcept      = default;

    void              register_subsystem(T& subsystem);
    SimSystemSlot<T>& operator=(T& subsystem) noexcept;
    SimSystemSlot<T>& operator=(T* subsystem) noexcept;

    T* view() const noexcept;
    T* operator->() const noexcept;
    operator bool() const noexcept;

  private:
    template <typename U>
    friend class SimSystemSlotCollection;

    SimSystemSlot(T& subsystem) noexcept;

    void       lazy_init() const;
    mutable T* m_subsystem = nullptr;
};

template <typename T>
class SimSystemSlotCollection
{
  public:
    void register_subsystem(T& subsystem);

    span<T* const> view() const noexcept;
    span<T*>       view() noexcept;

    template <typename U>
    SimSystemSlot<U> find() const noexcept;

  private:
    mutable list<T*>   m_subsystem_buffer;
    mutable vector<T*> m_subsystems;
    mutable bool       built = false;

    void lazy_init() const noexcept;
};
}  // namespace uipc::backend

#include "details/sim_system_slot.inl"