#include <uipc/backends/cuda/cuda_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>

namespace uipc::backend
{
void CudaEngine::do_init(backend::WorldVisitor v)
{
    spdlog::info("[CudaEngine] do_init() called.");
}

void CudaEngine::do_advance()
{
    spdlog::info("[CudaEngine] do_advance() called.");
}

void CudaEngine::do_sync()
{
    spdlog::info("[CudaEngine] do_sync() called.");
}

void CudaEngine::do_retrieve()
{
    spdlog::info("[CudaEngine] do_retrieve() called.");
}
}  // namespace uipc::backend


UIPC_BACKEND_API UIPCEngineInterface* uipc_create_engine()
{
    return new uipc::backend::CudaEngine();
}

UIPC_BACKEND_API void uipc_destroy_engine(UIPCEngineInterface* engine)
{
    delete engine;
}
