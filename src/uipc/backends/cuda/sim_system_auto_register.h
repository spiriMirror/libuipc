#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
namespace uipc::backend::cuda
{
class SimEngine;
class SimSystem;
class SimSystemAutoRegisterInternalData
{
  public:
    std::list<std::function<U<SimSystem>(SimEngine&)>> m_entries;
};
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
        [](::uipc::backend::cuda::SimEngine& engine)                               \
        { return std::make_unique<SimSystem>(engine); });

/**
 * @brief Register a SimSystem with advanced creation logic.
 *  
 * ```cpp
 * REGISTER_SIM_SYSTEM_ADVANCED(
 *  [](uipc::backend::cuda::SimEngine& engine)
 *  {
 *      if(need_to_create)
 *          return std::make_unique<SimSystem>(engine);
 *      else
 *          return nullptr;
 *  }
 * );
 * ```
 */
#define REGISTER_SIM_SYSTEM_ADVANCED                                           \
    static ::uipc::backend::cuda::SimSystemAutoRegister AutoRegister##__COUNTER__
