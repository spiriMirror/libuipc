#include <uipc/backends/cuda/sim_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>
#include <muda/muda.h>
#include <uipc/backends/cuda/kernel_cout.h>
#include <uipc/backends/cuda/sim_engine_device_common.h>
#include <uipc/backends/cuda/log_pattern_guard.h>
#include <uipc/backends/cuda/sim_system_collection.h>

namespace uipc::backend::cuda
{
void say_hello_from_muda()
{
    using namespace muda;

    Launch()
        .apply([] __device__()
               { cout << "Hello from CudaEngine CUDA Kernel!\n"; })
        .wait();
}

SimEngine::SimEngine()
    : m_device_common(std::make_unique<DeviceCommon>())
{

    LogGuard guard;

    spdlog::info("Cuda Backend Init Success.");

    using namespace muda;

    if(!cout)
    {
        auto viewer_ptr         = device_logger_viewer_ptr();
        m_device_common->logger = std::make_unique<muda::Logger>(viewer_ptr);

        Debug::set_sync_callback(
            [this]
            {
                m_string_stream.str("");
                m_device_common->logger->retrieve(m_string_stream);
                if(m_string_stream.str().empty())
                    return;

                std::string str = m_string_stream.str();
                spdlog::info(R"([Kernel Console] 
-------------------------------------------------------------------------------
{}
-------------------------------------------------------------------------------)",
                             str);
            });
    }

    say_hello_from_muda();
}

auto SimEngine::device_common() noexcept -> DeviceCommon&
{
    return *m_device_common;
}

WorldVisitor& SimEngine::world() noexcept
{
    UIPC_ASSERT(m_world_visitor, "WorldVisitor is not initialized.");
    return *m_world_visitor;
}

SimEngine::~SimEngine()
{
    LogGuard guard;

    muda::wait_device();

    // remove the sync callback
    muda::Debug::set_sync_callback(nullptr);
    cout = {};

    spdlog::info("Cuda Backend Shutdown Success.");
}
}  // namespace uipc::backend::cuda
