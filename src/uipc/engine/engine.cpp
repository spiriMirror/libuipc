#include <uipc/engine/engine.h>
#include <dylib.hpp>
#include <uipc/backend/module_init_info.h>
#include <filesystem>
namespace uipc::engine
{
static string to_lower(std::string_view s)
{
    string result{s};
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

class Engine::Impl
{
    std::string          m_backend_name;
    dylib                m_module;
    using Deleter                    = void (*)(IEngine*);
    IEngine*             m_engine    = nullptr;
    Deleter              m_deleter   = nullptr;
    mutable bool         m_sync_flag = false;
    std::string          m_workspace;

  public:
    Impl(std::string_view backend_name, std::string_view workspace)
        : m_backend_name(to_lower(backend_name))
        , m_module{fmt::format("uipc_backend_{}", m_backend_name)}
    {
        namespace fs = std::filesystem;

        m_workspace = fs::absolute(workspace).string();
        if(!fs::exists(m_workspace))  // create workspace
        {
            fs::create_directory(m_workspace);
        }
        else if(!fs::is_directory(m_workspace))
        {
            throw EngineException{fmt::format("Workspace [{}] is not a directory.", workspace)};
        }

        auto info              = make_unique<UIPCModuleInitInfo>();
        info->module_name      = m_backend_name;
        info->memory_resource  = std::pmr::get_default_resource();
        info->module_workspace = m_workspace;

        auto init = m_module.get_function<void(UIPCModuleInitInfo*)>("uipc_init_module");
        if(!init)
            throw EngineException{fmt::format("Can't find backend [{}]'s module initializer.",
                                              backend_name)};
        init(info.get());

        auto creator = m_module.get_function<IEngine*()>("uipc_create_engine");
        if(!creator)
            throw EngineException{fmt::format("Can't find backend [{}]'s engine creator.",
                                              backend_name)};
        m_engine = creator();
        m_deleter = m_module.get_function<void(IEngine*)>("uipc_destroy_engine");
        if(!m_deleter)
            throw EngineException{fmt::format("Can't find backend [{}]'s engine deleter.",
                                              backend_name)};
    }

    std::string_view backend_name() const { return m_backend_name; }

    void init(backend::WorldVisitor v) { m_engine->init(v); }

    void advance()
    {
        m_sync_flag = false;
        m_engine->advance();
    }

    void sync()
    {
        m_engine->sync();
        m_sync_flag = true;
    }

    void retrieve()
    {
        if(!m_sync_flag)
            m_engine->sync();
        m_engine->retrieve();
    }

    bool do_dump() { return m_engine->dump(); }

    bool do_recover() { return m_engine->recover(); }

    SizeT get_frame() const { return m_engine->frame(); }

    Json to_json() const
    {
        Json j;
        // force copy
        j = m_engine->to_json();
        return j;
    }

    ~Impl()
    {
        UIPC_ASSERT(m_deleter && m_engine, "Engine not initialized, why can it happen?");
        m_deleter(m_engine);
    }
};


Engine::Engine(std::string_view backend_name, std::string_view workspace)
    : m_impl{uipc::make_unique<Impl>(backend_name, workspace)}
{
}

Engine::~Engine() {}

void Engine::do_init(backend::WorldVisitor v)
{
    m_impl->init(v);
}

void Engine::do_advance()
{
    m_impl->advance();
}

void Engine::do_sync()
{
    m_impl->sync();
}

void Engine::do_retrieve()
{
    m_impl->retrieve();
}

Json Engine::do_to_json() const
{
    return m_impl->to_json();
}
bool Engine::do_dump()
{
    return m_impl->do_dump();
}
bool Engine::do_recover()
{
    return m_impl->do_recover();
}
SizeT Engine::get_frame() const
{
    return m_impl->get_frame();
}
}  // namespace uipc::engine
