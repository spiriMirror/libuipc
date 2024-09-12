#include <sim_engine.h>
#include <uipc/backends/common/module.h>
#include <uipc/backends/common/engine_create_info.h>

UIPC_BACKEND_API EngineInterface* uipc_create_engine(EngineCreateInfo* info)
{
    return new uipc::backend::cuda::SimEngine(info);
}

UIPC_BACKEND_API void uipc_destroy_engine(EngineInterface* engine)
{
    delete engine;
}