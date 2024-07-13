#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/common/type_traits.h>
#include <uipc/backends/common/i_sim_system.h>
#include <uipc/backends/common/sim_engine.h>

namespace uipc::backend
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
    {
        using Creator        = SimSystemCreator<SimSystemT>;
        using SignatureTuple = signature_t<decltype(Creator::create)>;
        using SimEngineT =
            std::remove_reference_t<std::tuple_element_t<1, SignatureTuple>>;
        using SimEnginePointer = std::add_pointer_t<SimEngineT>;

        if constexpr(std::is_same_v<SimEngineT, SimEngine>)
        {
            return [](SimEngine& engine) -> U<ISimSystem>
            {
                return ::uipc::static_pointer_cast<ISimSystem>(
                    SimSystemCreator<SimSystemT>::create(engine));
            };
        }
        else
        {
            return [](SimEngine& engine) -> U<ISimSystem>
            {
                SimEnginePointer e = dynamic_cast<SimEnginePointer>(&engine);
                UIPC_ASSERT(e != nullptr,
                            "{} cannot be cast to {}",
                            typeid(engine).name(),
                            typeid(SimEngineT).name());
                return ::uipc::static_pointer_cast<ISimSystem>(
                    SimSystemCreator<SimSystemT>::create(*e));
            };
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
}  // namespace uipc::backend

/**
 * @brief ConstitutionRegister a SimSystem, which will be automatically created by the SimEngine.
 */
#define REGISTER_SIM_SYSTEM(SimSystem)                                           \
    namespace auto_register                                                      \
    {                                                                            \
        static ::uipc::backend::SimSystemAutoRegister AutoRegister##__COUNTER__( \
            ::uipc::backend::detail::register_system_creator<SimSystem>());      \
    }

// End of file, remove the warning: backslash-newline at end of file