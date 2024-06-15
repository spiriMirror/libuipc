#pragma once
#include <uipc/common/macro.h>
#include <uipc/engine/engine.h>

using UIPCEngineInterface = uipc::engine::IEngine;

extern "C" {
UIPC_BACKEND_API UIPCEngineInterface* uipc_create_engine();
UIPC_BACKEND_API void uipc_destroy_engine(UIPCEngineInterface* engine);
}