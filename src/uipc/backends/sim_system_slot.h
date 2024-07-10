#pragma once
#include <uipc/backends/i_sim_system.h>

namespace uipc::backend
{
template <typename T>
class SimSystemSlotCollection
{
  public:
    void register_subsystem(T& subsystem);

    void init();

    span<T*>       view() noexcept;
    span<T* const> view() const noexcept;

  private:
    list<T*>     m_subsystem_buffer;
    vector<T*>   m_subsystems;
    mutable bool built = false;

    void check_build() const noexcept;
};

template <typename T>
class SimSystemSlot
{
  public:
    void register_subsystem(T& subsystem);

    void init();

    T*       view() noexcept;
    T* const view() const noexcept;

    T*       operator->() noexcept;
    T* const operator->() const noexcept;

    operator bool() const noexcept;

  private:
    mutable bool built       = false;
    T*           m_subsystem = nullptr;
    void         check_build() const noexcept;
};
}  // namespace uipc::backend

#include "details/sim_system_slot.inl"