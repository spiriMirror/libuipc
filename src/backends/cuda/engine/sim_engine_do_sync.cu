#include <sim_engine.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
void SimEngine::do_sync()
{
    try
    {
        // Sync the device
        cuda_tool::wait_device();
    }
    catch(const SimEngineException& e)
    {
        logger::error("SimEngine Sync Error: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}
}  // namespace uipc::backend::cuda
