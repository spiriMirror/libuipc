#include <sim_engine.h>
#include <backends/common/module.h>
#include <uipc/common/log.h>
#include <none_sim_system.h>
#include <uipc/common/log_pattern_guard.h>

namespace uipc::backend::none
{
SimEngine::SimEngine(EngineCreateInfo* info)
    : backend::SimEngine{info}
{
    LogPatternGuard guard("None");
    logger::info("Constructor called.");
    logger::info(R"(Hello, this is the NoneEngine from libuipc backends.
This engine does nothing, but helps to do the checking of the engine interface.
And it is a good place to print out some debug information during the life cycle of the engine.)");
}

void SimEngine::do_init(InitInfo& info)
{
    LogPatternGuard guard("None");
    logger::info("do_init() called.");

    build_systems();

    m_system = &require<NoneSimSystem>();

    dump_system_info();
}

void SimEngine::do_advance()
{
    LogPatternGuard guard("None");
    m_frame++;
    logger::info("do_advance() called.");
}

void SimEngine::do_sync()
{
    LogPatternGuard guard("None");
    logger::info("do_sync() called.");
}

void SimEngine::do_retrieve()
{
    LogPatternGuard guard("None");
    logger::info("do_receive() called.");
}

SimEngine::~SimEngine()
{
    LogPatternGuard guard("None");
    logger::info("Destructor called.");
}


SizeT SimEngine::get_frame() const
{
    return m_frame;
}
bool SimEngine::do_dump(DumpInfo&)
{
    // Now just do nothing
    return true;
}

bool SimEngine::do_try_recover(RecoverInfo&)
{
    // Now just do nothing
    return true;
}

void SimEngine::do_apply_recover(RecoverInfo& info)
{
    // If success, set the current frame to the recovered frame
    m_frame = info.frame();
}

void SimEngine::do_clear_recover(RecoverInfo& info)
{
    // If failed, do nothing
}
}  // namespace uipc::backend::none