#include <uipc/engine/engine.h>
#include <dylib.hpp>
#include <uipc/backend/module_init_info.h>
#include <uipc/backends/common/engine_create_info.h>
#include <uipc/common/uipc.h>
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
    using Deleter = void (*)(IEngine*);

    string   m_backend_name;
    S<dylib> m_module;

    IEngine*     m_engine    = nullptr;
    Deleter      m_deleter   = nullptr;
    mutable bool m_sync_flag = false;
    string       m_workspace;

    static unordered_map<string, S<dylib>> m_cache;

    static S<dylib> load_module(std::string_view backend_name)
    {
        // find in cache
        auto it = m_cache.find(std::string{backend_name});
        if(it != m_cache.end())
            return it->second;

        // if not found, load it
        auto& uipc_config = uipc::config();
        auto  backend =
            uipc::make_shared<dylib>(uipc_config["module_dir"].get<std::string>(),
                                     fmt::format("uipc_backend_{}", backend_name));

        auto info             = make_unique<UIPCModuleInitInfo>();
        info->module_name     = backend_name;
        info->memory_resource = std::pmr::get_default_resource();

        auto init = backend->get_function<void(UIPCModuleInitInfo*)>("uipc_init_module");
        if(!init)
            throw Exception{fmt::format("Can't find backend [{}]'s module initializer.",
                                        backend_name)};
        init(info.get());

        return m_cache[std::string{backend_name}] = backend;
    }

  public:
    Impl(std::string_view backend_name, std::string_view workspace, const Json& config)
        : m_backend_name(to_lower(backend_name))
    {
        namespace fs = std::filesystem;

        auto& uipc_config = uipc::config();

        m_module = load_module(m_backend_name);

        m_workspace = fs::absolute(workspace).string();
        if(!fs::exists(m_workspace))  // create workspace
        {
            fs::create_directory(m_workspace);
        }
        else if(!fs::is_directory(m_workspace))
        {
            throw EngineException{fmt::format("Workspace [{}] is not a directory.", workspace)};
        }

        auto creator = m_module->get_function<IEngine*(EngineCreateInfo*)>("uipc_create_engine");
        if(!creator)
            throw EngineException{fmt::format("Can't find backend [{}]'s engine creator.",
                                              backend_name)};

        EngineCreateInfo info;
        info.workspace = m_workspace;
        m_engine = creator(&info);
        m_deleter = m_module->get_function<void(IEngine*)>("uipc_destroy_engine");
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

unordered_map<string, S<dylib>> Engine::Impl::m_cache;


Engine::Engine(std::string_view backend_name, std::string_view workspace, const Json& config)
    : m_impl{uipc::make_unique<Impl>(backend_name, workspace, config)}
{
}

Engine::~Engine() {}

Json Engine::default_config()
{
    Json j = Json::object();
    return j;
}

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
