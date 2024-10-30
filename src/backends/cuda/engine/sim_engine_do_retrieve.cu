#include <sim_engine.h>

namespace uipc::backend::cuda
{
void SimEngine::do_retrieve()
{
    try
    {
        event_write_scene();
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("SimEngine Retrieve Error: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}

}  // namespace uipc::backend::cuda
