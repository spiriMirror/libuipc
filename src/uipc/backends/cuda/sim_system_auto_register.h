#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <type_traits>
#include <i_sim_system.h>

namespace uipc::backend::cuda
{
class SimEngine;

class SimSystemAutoRegisterInternalData
{
  public:
    list<std::function<U<ISimSystem>(SimEngine&)>> m_entries;
};


namespace detail
{
    template <typename SimSystemT>
    concept SimSystemHasAdvancedCreator = requires(SimEngine& engine) {
        {
            // 1) is derived from SimSystem
            std::is_base_of_v<ISimSystem, SimSystemT>&&
            // 2) has a static `advanced_creator` method
            SimSystemT::advanced_creator(engine)
        } -> std::convertible_to<U<ISimSystem>>;
    };

    template <std::derived_from<ISimSystem> SimSystemT>
    std::function<U<ISimSystem>(SimEngine&)> register_system_creator()
    {
        if constexpr(SimSystemHasAdvancedCreator<SimSystemT>)
        {
            return [](SimEngine& engine)
            { return SimSystemT::advanced_creator(engine); };
        }
        else
        {
            return [](SimEngine& engine)
            { return std::make_unique<SimSystemT>(engine); };
        }
    }
}  // namespace detail


class SimSystemAutoRegister
{
    friend class SimEngine;

  public:
    SimSystemAutoRegister(std::function<U<ISimSystem>(SimEngine&)>&& reg);

  private:
    static SimSystemAutoRegisterInternalData& internal_data();
};
}  // namespace uipc::backend::cuda

/**
 * @brief Register a SimSystem, which will be automatically created by the SimEngine.
 */

#define REGISTER_SIM_SYSTEM(SimSystem)                                             \
    static ::uipc::backend::cuda::SimSystemAutoRegister AutoRegister##__COUNTER__( \
        ::uipc::backend::cuda::detail::register_system_creator<SimSystem>());
