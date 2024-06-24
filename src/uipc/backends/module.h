#pragma once
#include <uipc/common/macro.h>
#include <memory_resource>
#include <uipc/engine/engine.h>
#include <uipc/backend/module_init_info.h>

using UIPCEngineInterface = uipc::engine::IEngine;

extern "C" {
UIPC_BACKEND_API UIPCEngineInterface* uipc_create_engine();
UIPC_BACKEND_API void uipc_destroy_engine(UIPCEngineInterface* engine);
UIPC_BACKEND_API void uipc_init_module(UIPCModuleInitInfo* info);
}