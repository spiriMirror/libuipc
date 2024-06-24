#include <uipc/engine/uipc_engine.h>
#include <dylib.hpp>
#include <uipc/backend/module_init_info.h>

namespace uipc::engine
{
static string to_lower(std::string_view s)
{
    string result{s};
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

class UIPCEngine::Impl
{
    string               m_backend_name;
    dylib                m_module;
    using Deleter                  = void (*)(IEngine*);
    IEngine*             m_engine  = nullptr;
    Deleter              m_deleter = nullptr;


  public:
    Impl(std::string_view backend_name)
        : m_backend_name(backend_name)
        , m_module{fmt::format("uipc_backend_{}", to_lower(backend_name))}
    {
        auto info = make_unique<UIPCModuleInitInfo>();

        m_module.get_function<void(UIPCModuleInitInfo*)>("uipc_init_module")(info.get());

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

    void advance() { m_engine->advance(); }

    void sync() { m_engine->sync(); }

    void retrieve() { m_engine->retrieve(); }

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


UIPCEngine::UIPCEngine(std::string_view backend_name)
    : m_impl{uipc::make_unique<Impl>(backend_name)}
{
}

UIPCEngine::~UIPCEngine() {}

void UIPCEngine::do_init(backend::WorldVisitor v)
{
    m_impl->init(v);
}

void UIPCEngine::do_advance()
{
    m_impl->advance();
}

void UIPCEngine::do_sync()
{
    m_impl->sync();
}

void UIPCEngine::do_retrieve()
{
    m_impl->retrieve();
}

Json UIPCEngine::do_to_json() const
{
    return m_impl->to_json();
}
}  // namespace uipc::engine
