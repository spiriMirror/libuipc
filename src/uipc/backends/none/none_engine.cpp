#include <uipc/backends/none/none_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>

namespace uipc::backend
{
NoneEngine::NoneEngine()
{
    spdlog::info("[NoneEngine] Constructor called.");
    spdlog::info(R"(Hello, this is the NoneEngine from libuipc backends.
This engine does nothing, but helps to do the checking of the engine interface.
And it is a good place to print out some debug information during the life cycle of the engine.
)");
}

void NoneEngine::do_init(backend::WorldVisitor v)
{
    spdlog::info("[NoneEngine] do_init() called.");
}

void NoneEngine::do_advance()
{
    spdlog::info("[NoneEngine] do_advance() called.");
}

void NoneEngine::do_sync()
{
    spdlog::info("[NoneEngine] do_sync() called.");
}

void NoneEngine::do_retrieve()
{
    spdlog::info("[NoneEngine] do_retrieve() called.");
}

NoneEngine::~NoneEngine()
{
    spdlog::info("[NoneEngine] Destructor called.");
}
}  // namespace uipc::backend


UIPC_BACKEND_API UIPCEngineInterface* uipc_create_engine()
{
    return new uipc::backend::NoneEngine();
}

UIPC_BACKEND_API void uipc_destroy_engine(UIPCEngineInterface* engine)
{
    delete engine;
}
