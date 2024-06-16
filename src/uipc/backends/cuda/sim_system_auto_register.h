#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <type_traits>

namespace uipc::backend::cuda
{
class SimEngine;
class SimSystem;
class SimSystemAutoRegisterInternalData
{
  public:
    std::list<std::function<U<SimSystem>(SimEngine&)>> m_entries;
};


namespace detail
{
    template <typename SimSystemT>
    concept SimSystemHasAdvancedCreator = requires(SimEngine& engine) {
        {
            // 1) is derived from SimSystem
            std::is_base_of_v<SimSystem, SimSystemT>&&
            // 2) has a static `advanced_creator` method
            SimSystemT::advanced_creator(engine)
        } -> std::convertible_to<U<SimSystem>>;
    };

    template <std::derived_from<SimSystem> SimSystemT>
    std::function<U<SimSystem>(SimEngine&)> register_system_creator()
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
    SimSystemAutoRegister(std::function<U<SimSystem>(SimEngine&)>&& reg);

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
