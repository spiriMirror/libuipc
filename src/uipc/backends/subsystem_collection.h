#pragma once
#include <concepts>
#include <uipc/backends/i_sim_system.h>

namespace uipc::backend
{
template <std::derived_from<ISimSystem> T>
class SubsystemCollection
{
  public:
    void register_subsystem(T* subsystem);

    void build();

    span<T*>       view() noexcept;
    span<T* const> view() const noexcept;

  private:
    list<T*>     m_subsystem_buffer;
    vector<T*>   m_subsystems;
    mutable bool built = false;

    void check_build() const noexcept;
};
}  // namespace uipc::backend

#include "details/subsystem_collection.inl"