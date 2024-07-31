#pragma once
#include <uipc/common/macro.h>
#include <memory_resource>
#include <uipc/engine/engine.h>
#include <uipc/backend/module_init_info.h>

using EngineInterface = uipc::engine::IEngine;

extern "C" {
/**
 * @brief Create a new engine instance.
 * 
 * This function is used to create a new engine instance, the module **should** implement this function.
 *  
 * @return A pointer to the new engine instance.
 */
UIPC_BACKEND_API EngineInterface* uipc_create_engine();

/**
 * @brief Destroy the engine instance.
 * 
 * This function is used to destroy the engine instance, the module **should** implement this function.
 * 
 * @param engine The engine instance to be destroyed.
 */
UIPC_BACKEND_API void uipc_destroy_engine(EngineInterface* engine);

/**
 * @brief Initialize the module with the given information.
 * 
 * This function is already implemented in the <uipc/backends/module.cpp> file.
 * 
 * @param info The module initialization information.
*/
UIPC_BACKEND_API void uipc_init_module(UIPCModuleInitInfo* info);
}

namespace uipc::backend
{
class ModuleInfo
{
  public:
    std::string_view   workspace() const noexcept { return m_workspace; }
    static ModuleInfo& instance() noexcept;
    void               init(const UIPCModuleInitInfo& info) noexcept;

  private:
    std::string m_workspace;
};
}  // namespace uipc::backend