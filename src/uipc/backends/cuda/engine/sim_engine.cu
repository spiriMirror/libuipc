#include <sim_engine.h>
#include <uipc/common/log.h>
#include <muda/muda.h>
#include <kernel_cout.h>
#include <sim_engine_device_common.h>
#include <log_pattern_guard.h>
#include <uipc/backends/common/module.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_surface_manager.h>
#include <fstream>

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

        spdlog::info("Initializing Cuda Backend...");

        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        spdlog::info("Device: {}", prop.name);
        spdlog::info("Compute Capability: {}.{}", prop.major, prop.minor);

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
        spdlog::info("Cuda Backend Init Success.");
    }
    catch(const SimEngineException& e)
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

void uipc::backend::cuda::SimEngine::dump_global_surface(std::string_view name)
{
    auto path      = ModuleInfo::instance().workspace();
    auto file_path = fmt::format("{}{}.obj", path, name);

    std::vector<Vector3> positions;
    auto                 src_ps = m_global_vertex_manager->positions();
    positions.resize(src_ps.size());
    src_ps.copy_to(positions.data());

    std::vector<Vector3i> faces;
    auto                  src_fs = m_global_surface_manager->surf_triangles();
    faces.resize(src_fs.size());
    src_fs.copy_to(faces.data());

    std::ofstream file(file_path);

    for(auto& pos : positions)
        file << fmt::format("v {} {} {}\n", pos.x(), pos.y(), pos.z());

    for(auto& face : faces)
        file << fmt::format("f {} {} {}\n", face.x() + 1, face.y() + 1, face.z() + 1);

    spdlog::info("Dumped global surface to {}", file_path);
}

bool SimEngine::do_dump()
{
    auto path = dump_path();

    Json j     = Json::object();
    j["frame"] = m_current_frame;

    {
        std::ofstream file(path + "state.json");
        file << j.dump(4);
    }

    return backend::SimEngine::do_dump();
}

bool SimEngine::do_recover()
{
    auto path = dump_path();

    bool success = false;

    do
    {
        Json j;
        {
            std::ifstream file(path + "state.json");
            if(file.is_open())
            {
                file >> j;
            }
            else
            {
                spdlog::warn("Failed to open state.json when recovering, so skip.");
                break;
            }
        }

        if(!backend::SimEngine::do_recover())
            break;

        bool has_error = false;
        try
        {
            j["frame"].get<SizeT>();
        }
        catch(std::exception e)
        {
            has_error = true;
            spdlog::warn("Failed to retrieve data from state.json when recovering, so skip. Reason: {}",
                         e.what());
        }
        if(has_error)
            break;

        m_current_frame = j["frame"];
        spdlog::info("Recover at frame: {}", m_current_frame);

        success = true;
    } while(0);

    return success;
}
}  // namespace uipc::backend::cuda
