#include <uipc/backends/cuda/sim_system.h>
#include <typeinfo>
#include <uipc/backends/cuda/sim_engine.h>

namespace uipc::backend::cuda
{
SimSystem::SimSystem(SimEngine& sim_engine) noexcept
    : m_sim_engine(sim_engine)
{
}

std::string_view SimSystem::name() const noexcept
{
    return get_name();
}

SimEngine& uipc::backend::cuda::SimSystem::engine() noexcept
{
    return m_sim_engine;
}

const SimEngine& SimSystem::engine() const noexcept
{
    return m_sim_engine;
}

std::string_view SimSystem::get_name() const noexcept
{
    return typeid(*this).name();
}
SimSystemCollection& SimSystem::collection()
{
    return m_sim_engine.m_system_collection;
}
}  // namespace uipc::backend::cuda
