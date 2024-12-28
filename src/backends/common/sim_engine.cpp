#include <backends/common/sim_engine.h>
#include <backends/common/sim_system_auto_register.h>
#include <backends/common/module.h>
#include <filesystem>
#include <fstream>
#include <uipc/backend/engine_create_info.h>
#include <backends/common/backend_path_tool.h>

namespace uipc::backend
{
SimEngine::SimEngine(EngineCreateInfo* info)
    : m_workspace(info->workspace)
{
}

Json SimEngine::do_to_json() const
{
    Json j;
    j["sim_systems"] = m_system_collection.to_json();
    return j;
}

WorldVisitor& SimEngine::world() noexcept
{
    UIPC_ASSERT(m_world_visitor, "WorldVisitor is not initialized.");
    return *m_world_visitor;
}

void SimEngine::build_systems()
{
    auto& funcs = SimSystemAutoRegister::creators().entries;
    for(auto& f : funcs)
    {
        auto uptr = f(*this);
        if(uptr)
            m_system_collection.create(std::move(uptr));
    }

    m_system_collection.build_systems();
}

void SimEngine::dump_system_info() const
{
    namespace fs = std::filesystem;

    spdlog::debug("Built systems:\n{}", m_system_collection);

    fs::path p = fs::absolute(fs::path{workspace()} / "systems.json");
    {
        std::ofstream ofs(p);
        ofs << to_json().dump(4);
    }
    spdlog::info("System info dumped to {}", p.string());
}

std::string SimEngine::dump_path(std::string_view _file_) const noexcept
{
    BackendPathTool tool{workspace()};
    return tool.workspace(_file_, "dump").string();
}

span<ISimSystem* const> SimEngine::systems() noexcept
{
    return m_system_collection.systems();
}

core::EngineStatusCollection& SimEngine::get_status() noexcept
{
    return m_status;
}

ISimSystem* SimEngine::find_system(ISimSystem* ptr)
{
    if(ptr)
    {
        if(!ptr->is_valid())
        {
            ptr = nullptr;
        }
        else
        {
            ptr->set_engine_aware();
        }
    }
    return ptr;
}

ISimSystem* SimEngine::require_system(ISimSystem* ptr)
{
    if(ptr)
    {
        if(!ptr->is_valid())
        {
            throw SimEngineException(fmt::format("SimSystem [{}] is invalid", ptr->name()));
        }
        else
        {
            ptr->set_engine_aware();
        }
    }
    return ptr;
}

std::string_view SimEngine::workspace() const noexcept
{
    return m_workspace;
}

static constexpr std::string_view dump_file_name = "state";
static constexpr std::string_view dump_file_ext  = "json";

static bool find_max_dump_frame(std::string_view path, SizeT& max_frame)
{
    namespace fs = std::filesystem;

    bool found = false;
    max_frame  = 0;

    for(auto& p : fs::directory_iterator(path))
    {
        if(p.is_regular_file())
        {
            std::string file_name       = p.path().filename().string();
            SizeT       file_name_begin = file_name.find(dump_file_name);
            if(file_name_begin == std::string::npos)
                continue;
            SizeT frame_str_begin = file_name_begin + dump_file_name.size() + 1;  // +1 for the dot
            SizeT frame_str_end = file_name.find(fmt::format(".{}", dump_file_ext));
            if(frame_str_end == std::string::npos)
                continue;

            auto frame_str =
                file_name.substr(frame_str_begin, frame_str_end - frame_str_begin);

            SizeT frame = std::stoull(frame_str);

            if(!found)
            {
                found     = true;
                max_frame = frame;
            }
            else
            {
                max_frame = std::max(max_frame, frame);
            }
        }
    }

    return found;
}


bool SimEngine::do_dump()
{
    auto path = dump_path(__FILE__);

    BackendPathTool tool{workspace()};
    auto            current_frame = frame();
    auto            backend_name  = tool.backend_name();

    // 1. Dump SimEngine
    {
        Json j       = Json::object();
        j["frame"]   = current_frame;
        j["backend"] = backend_name;

        // 1.1 Write to dump file
        bool success = true;
        try
        {
            std::ofstream file(fmt::format("{}{}.{}.json", path, dump_file_name, current_frame));
            file << j.dump(4);
        }
        catch(std::exception e)
        {
            spdlog::error("Failed to write to dump file. Reason: {}", e.what());
            success = false;
        }

        if(!success)
            return false;

        // 1.2 Let the subclass to dump
        try
        {
            DumpInfo dump_info{frame(), workspace(), Json::object()};
            success = do_dump(dump_info);
        }
        catch(std::exception e)
        {
            spdlog::error("Failed to dump engine. Reason: {}", e.what());
            success = false;
        }

        if(!success)
            return false;
    }


    // 2. Dump subsystems
    bool all_success = true;
    for(auto system : systems())
    {
        ISimSystem::DumpInfo info{frame(), workspace(), Json::object()};
        all_success &= system->do_dump(info);

        if(!all_success)
        {
            spdlog::error("Failed to dump system [{}]", system->name());
            break;
        }
    }

    return all_success;
}

void SimEngine::do_init(backend::WorldVisitor v)
{
    m_world_visitor = make_unique<backend::WorldVisitor>(v);

    InitInfo init_info{Json::object()};
    do_init(init_info);
}

bool SimEngine::do_recover(SizeT dst_frame)
{
    auto            path = dump_path(__FILE__);
    BackendPathTool tool{workspace()};

    const Json& info              = world().scene().info();
    SizeT       try_recover_frame = dst_frame;
    auto        backend_name      = tool.backend_name();

    // 1. Get the file path
    std::string dump_file_path;
    {
        if(try_recover_frame == ~0ull)  // Try to find the max frame
        {
            SizeT max_frame;
            if(find_max_dump_frame(path, max_frame))
            {
                try_recover_frame = max_frame;
            }
            else
            {
                spdlog::info("No dump files found, so skip recovery.");
                return false;
            }
        }
        dump_file_path =
            fmt::format("{}{}.{}.json", path, dump_file_name, try_recover_frame);
    }

    // 2. Check if the dump file is valid
    {
        std::ifstream ifs(dump_file_path);
        if(!ifs)
        {
            spdlog::info("No dump file {} found, so skip recovery.", dump_file_path);
            return false;
        }

        Json j;
        {
            std::ifstream ifs(dump_file_path);
            if(!ifs)
            {
                spdlog::info("No dump file {} found, so skip recovery.", dump_file_path);
                return false;
            }
            ifs >> j;
        }

        bool        has_error   = false;
        SizeT       check_frame = ~0ull;
        std::string check_backend_name;
        try
        {
            check_frame        = j["frame"].get<SizeT>();
            check_backend_name = j["backend"].get<std::string>();
        }
        catch(std::exception e)
        {
            has_error = true;
            spdlog::info("Failed to retrieve data from state.json when recovering, so skip. Reason: {}",
                         e.what());
        }
        if(has_error)
        {
            spdlog::info("Failed to recover from dump file {}, so skip recovery.", dump_file_path);
            return false;
        }
        if(check_frame != try_recover_frame)
        {
            spdlog::info("Frame mismatch when recovering, so skip recovery. try={}, record={}",
                         try_recover_frame,
                         check_frame);
            return false;
        }
        if(check_backend_name != backend_name)
        {
            spdlog::info("Backend name mismatch when recovering, so skip recovery. try={}, record={}",
                         backend_name,
                         check_backend_name);
            return false;
        }
    }

    bool all_success = true;
    {
        RecoverInfo engine_recover_info{try_recover_frame, workspace(), Json::object()};
        ISimSystem::RecoverInfo simsystem_recover_info{
            try_recover_frame, workspace(), Json::object()};

        all_success &= this->do_try_recover(engine_recover_info);
        if(!all_success)
        {
            spdlog::warn("Try recovering engine fails, so skip recovery.");
        }
        else
        {
            for(auto system : systems())
            {
                all_success &= system->try_recover(simsystem_recover_info);

                if(!all_success)
                {
                    spdlog::warn("Try recovering system [{}] fails, so skip recovery.",
                                 system->name());
                    break;
                }
            }
        }

        if(all_success)  // If all success, apply recover
        {
            this->do_apply_recover(engine_recover_info);
            for(auto system : systems())
                system->apply_recover(simsystem_recover_info);
        }
        else  // If any fails, clear all recover
        {
            this->do_clear_recover(engine_recover_info);
            for(auto system : systems())
                system->clear_recover(simsystem_recover_info);
        }
    }

    if(all_success)
    {
        spdlog::info("Successfully recovered to frame {}", try_recover_frame);
    }

    return all_success;
}
}  // namespace uipc::backend