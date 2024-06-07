#include <uipc/backends/cuda/cuda_engine.h>
#include <uipc/backends/module.h>
#include <uipc/common/log.h>
#include <muda/muda.h>
#include <uipc/backends/cuda/kernel_cout.h>
#include <uipc/backends/cuda/cuda_engine_device_common.h>

namespace uipc::backend
{
void say_hello_from_muda()
{
    using namespace muda;

    Launch()
        .apply([] __device__()
               { cout << "Hello from CudaEngine CUDA Kernel! \n "; })
        .wait();
}

CudaEngine::CudaEngine()
    : m_device_common(std::make_unique<DeviceCommon>())
{
    spdlog::info("[CudaEngine] Cuda Backend Init Success.");

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
                if(str.back() == '\n')
                    str.pop_back();
                spdlog::info(R"([CudaEngine Kernel Console]:
-------------------------------------------------------------------------------
{}
-------------------------------------------------------------------------------)",
                             str);
            });
    }

    say_hello_from_muda();
}

auto CudaEngine::device_common() noexcept -> DeviceCommon&
{
    return *m_device_common;
}

CudaEngine::~CudaEngine()
{
    muda::wait_device();

    // remove the sync callback
    muda::Debug::set_sync_callback(nullptr);
    cout = {};

    spdlog::info("[CudaEngine] Cuda Backend Shutdown Success.");
}
}  // namespace uipc::backend
