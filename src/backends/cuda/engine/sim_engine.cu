#include <sim_engine.h>
#include <uipc/common/log.h>
#include <muda/muda.h>
#include <kernel_cout.h>
#include <sim_engine_device_common.h>
#include <backends/common/module.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <fstream>
#include <uipc/common/timer.h>
#include <backends/common/backend_path_tool.h>
#include <uipc/backend/engine_create_info.h>
#include <future>

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

SimEngine::SimEngine(EngineCreateInfo* info)
    : backend::SimEngine(info)
    , m_device_impl(make_unique<DeviceImpl>())
{
    try
    {
        using namespace muda;

        spdlog::info("Initializing Cuda Backend...");

        auto device_id = info->config["gpu"]["device"].get<IndexT>();

        // get gpu device count
        int device_count;
        checkCudaErrors(cudaGetDeviceCount(&device_count));
        if(device_id >= device_count)
        {
            UIPC_WARN_WITH_LOCATION(
                "Cannot find device with id {}. Using device 0 instead.", 
                device_id);

            device_id = 0;
        }

        cudaDeviceProp prop;
        checkCudaErrors(cudaGetDeviceProperties(&prop, device_id));
        spdlog::info("Device: [{}] {}", device_id, prop.name);
        spdlog::info("Compute Capability: {}.{}", prop.major, prop.minor);
        spdlog::info("Total Global Memory: {} MB", prop.totalGlobalMem / 1024 / 1024);

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

        Timer::set_sync_func([] { muda::wait_device(); });

#ifndef NDEBUG
        // if in debug mode, sync all the time to check for errors
        muda::Debug::debug_sync_all(true);
#endif

        spdlog::info("Cuda Backend Init Success.");
    }
    catch(const SimEngineException& e)
    {
        spdlog::error("Cuda Backend Init Failed: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}

SimEngine::~SimEngine()
{
    muda::wait_device();

    // remove the sync callback
    muda::Debug::set_sync_callback(nullptr);

    spdlog::info("Cuda Backend Shutdown Success.");
}

auto SimEngine::device_impl() noexcept -> DeviceImpl&
{
    return *m_device_impl;
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

void SimEngine::dump_global_surface(std::string_view name)
{
    BackendPathTool tool{workspace()};
    auto file_path = fmt::format("{}/{}.obj", tool.workspace().string(), name);

    std::vector<Vector3> positions;
    std::vector<Vector3> disps;

    auto src_ps = m_global_vertex_manager->prev_positions();
    auto disp   = m_global_vertex_manager->displacements();
    positions.resize(src_ps.size());
    src_ps.copy_to(positions.data());
    disps.resize(disp.size());
    disp.copy_to(disps.data());

    std::ranges::transform(positions, disps, positions.begin(), std::plus<>());

    std::vector<Vector2i> edges;
    auto src_es = m_global_simplicial_surface_manager->surf_edges();
    edges.resize(src_es.size());
    src_es.copy_to(edges.data());

    std::vector<Vector3i> faces;
    auto src_fs = m_global_simplicial_surface_manager->surf_triangles();
    faces.resize(src_fs.size());
    src_fs.copy_to(faces.data());

    std::ofstream file(file_path);

    for(auto& pos : positions)
        file << fmt::format("v {} {} {}\n", pos.x(), pos.y(), pos.z());

    for(auto& face : faces)
        file << fmt::format("f {} {} {}\n", face.x() + 1, face.y() + 1, face.z() + 1);

    for(auto& edge : edges)
        file << fmt::format("l {} {}\n", edge.x() + 1, edge.y() + 1);

    spdlog::info("Dumped global surface to {}", file_path);
}
}  // namespace uipc::backend::cuda

// Dump & Recover:
namespace uipc::backend::cuda
{
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
    m_current_frame = info.frame();
}

void SimEngine::do_clear_recover(RecoverInfo& info)
{
    // If failed, do nothing
}

SizeT SimEngine::get_frame() const
{
    return m_current_frame;
}
}  // namespace uipc::backend::cuda
