#include <sim_engine.h>
#include <uipc/backends/common/module.h>

UIPC_BACKEND_API EngineInterface* uipc_create_engine()
{
    return new uipc::backend::cuda::SimEngine();
}

UIPC_BACKEND_API void uipc_destroy_engine(EngineInterface* engine)
{
    delete engine;
}