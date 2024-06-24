#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <type_traits>
#include <i_sim_system.h>
#include <type_traits>

namespace uipc::backend::cuda
{
class SimEngine;

class SimSystemAutoRegisterInternalData
{
  public:
    list<std::function<U<ISimSystem>(SimEngine&)>> m_entries;
};

// default creator
template <std::derived_from<ISimSystem> SimSystemT>
class SimSystemCreator
{
  public:
    static U<SimSystemT> create(SimEngine& engine)
    {
        return ::uipc::make_unique<SimSystemT>(engine);
    }
};

namespace detail
{
    template <std::derived_from<ISimSystem> SimSystemT>
    std::function<U<ISimSystem>(SimEngine&)> register_system_creator()
        requires requires(SimEngine& engine) {
            {
                SimSystemCreator<SimSystemT>::create(engine)
            } -> std::same_as<U<SimSystemT>>;
        }
    {
        return [](SimEngine& engine) -> U<ISimSystem>
        {
            return ::uipc::static_pointer_cast<ISimSystem>(
                SimSystemCreator<SimSystemT>::create(engine));
        };
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
 * @brief ConstitutionRegister a SimSystem, which will be automatically created by the SimEngine.
 */

#define REGISTER_SIM_SYSTEM(SimSystem)                                             \
    static ::uipc::backend::cuda::SimSystemAutoRegister AutoRegister##__COUNTER__( \
        ::uipc::backend::cuda::detail::register_system_creator<SimSystem>());
