#pragma once
#include <uipc/common/dllexport.h>
#include <memory_resource>
#include <uipc/core/i_engine.h>

using EngineInterface = uipc::core::IEngine;
class UIPCModuleInitInfo;
namespace uipc::backend
{
class EngineCreateInfo;
}
using EngineCreateInfo = uipc::backend::EngineCreateInfo;

extern "C" {
/**
 * @brief Create a new engine instance.
 * 
 * This function is used to create a new engine instance, the module **should** implement this function.
 *  
 * @return A pointer to the new engine instance.
 */
UIPC_BACKEND_API EngineInterface* uipc_create_engine(EngineCreateInfo* info);

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