#include <sim_engine.h>
#include <uipc/common/log.h>
#include <muda/muda.h>
#include <kernel_cout.h>
#include <backends/common/module.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <uipc/common/timer.h>
#include <backends/common/backend_path_tool.h>
#include <uipc/backend/engine_create_info.h>

namespace uipc::backend::cuda
{
void say_hello_from_cuda()
{
    using namespace muda;
    Launch()
        .apply([cout = KernelCout::viewer()] __device__() mutable
               { cout << "CUDA Backend Kernel Console Init Success!\n"; })
        .wait();
}

SimEngine::SimEngine(EngineCreateInfo* info)
    : backend::SimEngine(info)
{
    try
    {
        using namespace muda;

        logger::info("Initializing Cuda Backend...");

        auto device_id = info->config["gpu"]["device"].get<IndexT>();

        // get gpu device count
        int device_count;
        checkCudaErrors(cudaGetDeviceCount(&device_count));
        if(device_id >= device_count)
        {
            UIPC_WARN_WITH_LOCATION("Cannot find device with id {}. Using device 0 instead.",
                                    device_id);

            device_id = 0;
        }

        cudaDeviceProp prop;
        checkCudaErrors(cudaGetDeviceProperties(&prop, device_id));
        logger::info("Device: [{}] {}", device_id, prop.name);
        logger::info("Compute Capability: {}.{}", prop.major, prop.minor);
        logger::info("Total Global Memory: {} MB", prop.totalGlobalMem / 1024 / 1024);

        Timer::set_sync_func([] { muda::wait_device(); });

        say_hello_from_cuda();

#ifndef NDEBUG
        // if in debug mode, sync all the time to check for errors
        muda::Debug::debug_sync_all(true);
#endif
        logger::info("Cuda Backend Init Success.");
    }
    catch(const SimEngineException& e)
    {
        logger::error("Cuda Backend Init Failed: {}", e.what());
        status().push_back(core::EngineStatus::error(e.what()));
    }
}

SimEngine::~SimEngine()
{
    muda::wait_device();

    // remove the sync callback
    muda::Debug::set_sync_callback(nullptr);

    logger::info("Cuda Backend Shutdown Success.");
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

void SimEngine::dump_global_surface()
{
    BackendPathTool tool{workspace()};
    auto            output_folder = tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    auto            file_path = fmt::format("{}global_surface.{}.{}.{}.obj",
                                 output_folder.string(),
                                 frame(),
                                 newton_iter(),
                                 line_search_iter());

    std::vector<Vector3> positions;
    std::vector<Vector3> disps;

    auto src_ps = m_global_vertex_manager->positions();

    positions.resize(src_ps.size());
    src_ps.copy_to(positions.data());

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

    logger::info("Dumped global surface to {}", file_path);
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

SizeT SimEngine::newton_iter() const noexcept
{
    return m_newton_iter;
}

SizeT SimEngine::line_search_iter() const noexcept
{
    return m_line_search_iter;
}
}  // namespace uipc::backend::cuda
