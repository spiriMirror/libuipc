#pragma once
#include <string_view>
#include <uipc/backends/cuda/sim_system_auto_register.h>

namespace uipc::backend::cuda
{
class SimEngine;
class SimSystemCollection;
class SimSystem
{
    friend class SimEngine;

  public:
    SimSystem(SimEngine& sim_engine) noexcept;
    virtual ~SimSystem() = default;
    std::string_view name() const noexcept;

  protected:
    template <std::derived_from<SimSystem> T>
    T* find();

    virtual void build() noexcept {};

    SimEngine&               engine() noexcept;
    const SimEngine&         engine() const noexcept;
    virtual std::string_view get_name() const noexcept;

  private:
    SimEngine& m_sim_engine;
    SimSystemCollection& collection();
};
}  // namespace uipc::backend::cuda

#include "details/sim_system.inl"