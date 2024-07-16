#include <sim_engine.h>
#include <uipc/common/log.h>
#include <muda/muda.h>
#include <kernel_cout.h>
#include <sim_engine_device_common.h>
#include <log_pattern_guard.h>

namespace uipc::backend::cuda
{
void say_hello_from_muda()
{
    using namespace muda;

    Launch()
        .apply([] __device__()
               { cout << "CUDA Backend Kernel Console Init Success!\n"; })
        .wait();
}

SimEngine::SimEngine()
    : m_device_impl(make_unique<DeviceImpl>())
{
    LogGuard guard;
    try
    {
        using namespace muda;

        spdlog::info("Cuda Backend Init Success.");

        auto viewer_ptr       = device_logger_viewer_ptr();
        m_device_impl->logger = make_unique<muda::Logger>(viewer_ptr);

        Debug::set_sync_callback(
            [this]
            {
                m_string_stream.str("");
                m_device_impl->logger->retrieve(m_string_stream);
                if(m_string_stream.str().empty())
                    return;

                spdlog::info(R"( 
-------------------------------------------------------------------------------
*                               Kernel  Console                               *
-------------------------------------------------------------------------------
{}
-------------------------------------------------------------------------------)",
                             m_string_stream.str());
            });

        say_hello_from_muda();

#ifndef NDEBUG
        // if in debug mode, sync all the time to check for errors
        muda::Debug::debug_sync_all(true);
#endif
    }
    catch(SimEngineException& e)
    {
        spdlog::error("Cuda Backend Init Failed: {}", e.what());
    }
}

SimEngine::~SimEngine()
{
    LogGuard guard;

    muda::wait_device();

    // remove the sync callback
    muda::Debug::set_sync_callback(nullptr);

    spdlog::info("Cuda Backend Shutdown Success.");
}

auto SimEngine::device_impl() noexcept -> DeviceImpl&
{
    return *m_device_impl;
}

WorldVisitor& SimEngine::world() noexcept
{
    UIPC_ASSERT(m_world_visitor, "WorldVisitor is not initialized.");
    return *m_world_visitor;
}

SimEngineState SimEngine::state() const noexcept
{
    return m_state;
}

void SimEngine::event_init_scene()
{
    for(auto& action : m_on_init_scene.view())
        action();
}

void SimEngine::event_rebuild_scene()
{
    for(auto& action : m_on_rebuild_scene.view())
        action();
}

void SimEngine::event_write_scene()
{
    for(auto& action : m_on_write_scene.view())
        action();
}
}  // namespace uipc::backend::cuda
