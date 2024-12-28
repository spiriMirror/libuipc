#include <uipc/core/sanity_checker.h>
#include <dylib.hpp>
#include <uipc/common/uipc.h>
#include <uipc/backend/module_init_info.h>
#include <uipc/core/scene.h>
#include <uipc/core/world.h>
#include <uipc/core/engine.h>

namespace uipc::core
{
static S<dylib> load_sanity_check_module()
{
    static std::mutex m_cache_mutex;
    static S<dylib>   m_cache;

    std::lock_guard lock{m_cache_mutex};

    if(m_cache)
        return m_cache;

    // if not found, load it
    auto& uipc_config = uipc::config();
    auto  this_module =
        uipc::make_shared<dylib>(uipc_config["module_dir"].get<std::string>(),
                                 "uipc_sanity_check");

    std::string_view module_name = "sanity_check";

    UIPCModuleInitInfo info;
    info.module_name     = "sanity_check";
    info.memory_resource = std::pmr::get_default_resource();

    auto init = this_module->get_function<void(UIPCModuleInitInfo*)>("uipc_init_module");
    if(!init)
        throw Exception{fmt::format("Can't find [sanity_check]'s module initializer.")};

    init(&info);

    m_cache = this_module;
    return m_cache;
}

SanityChecker::SanityChecker(Scene& scene)
    : m_scene{scene}
{
}

SanityChecker::~SanityChecker() {}

SanityCheckResult SanityChecker::check(std::string_view workspace)
{
    clear();

    auto sanity_check_module = load_sanity_check_module();
    auto creator =
        sanity_check_module->get_function<ISanityCheckerCollection*(SanityCheckerCollectionCreateInfo*)>(
            "uipc_create_sanity_checker_collection");

    if(!creator)
    {
        spdlog::error("Can't find [sanity_check]'s sanity checker creator, so we skip sanity check.");
        return SanityCheckResult::Error;
    }

    auto destroyer = sanity_check_module->get_function<void(ISanityCheckerCollection*)>(
        "uipc_destroy_sanity_checker_collection");

    if(!destroyer)
    {
        spdlog::error("Can't find [sanity_check]'s sanity checker destroyer, so we skip sanity check.");
        return SanityCheckResult::Error;
    }

    SanityCheckerCollectionCreateInfo info;
    info.workspace = workspace;

    ISanityCheckerCollection* sanity_checkers = creator(&info);
    sanity_checkers->build(m_scene);

    SanityCheckMessageCollection msgs;

    auto result = sanity_checkers->check(msgs);

    for(const auto& [id, msg] : msgs.messages())
    {
        if(msg->is_empty())
            continue;

        switch(msg->result())
        {
            case SanityCheckResult::Error:
                m_errors.messages().insert({id, msg});
                break;
            case SanityCheckResult::Warning:
                m_warns.messages().insert({id, msg});
                break;
            case SanityCheckResult::Success:
                m_infos.messages().insert({id, msg});
                break;
            default:
                break;
        }
    }

    spdlog::info("SanityCheck Summary: {} errors, {} warns, {} infos",
                 m_errors.messages().size(),
                 m_warns.messages().size(),
                 m_infos.messages().size());

    destroyer(sanity_checkers);

    return result;
}

void SanityChecker::report()
{
    std::string buffer;

    for(const auto& [id, msg] : m_errors.messages())
    {
        fmt::format_to(std::back_inserter(buffer), "[{}({})]:\n{}\n", msg->name(), id, msg->message());
        fmt::format_to(std::back_inserter(buffer), "Geometries:\n");
        for(const auto& [name, geo] : msg->geometries())
        {
            fmt::format_to(std::back_inserter(buffer), "  - {}: <{}>\n", name, geo->type());
        }
        buffer.pop_back();  // remove the last '\n'
        spdlog::error(buffer);
        buffer.clear();
    }

    for(const auto& [id, msg] : m_warns.messages())
    {
        fmt::format_to(std::back_inserter(buffer), "[{}({})]:\n{}\n", msg->name(), id, msg->message());
        fmt::format_to(std::back_inserter(buffer), "Geometries:\n");
        for(const auto& [name, geo] : msg->geometries())
        {
            fmt::format_to(std::back_inserter(buffer), "  - {}: <{}>\n", name, geo->type());
        }
        buffer.pop_back();  // remove the last '\n'
        spdlog::warn(buffer);
        buffer.clear();
    }

    for(const auto& [id, msg] : m_infos.messages())
    {
        fmt::format_to(std::back_inserter(buffer), "[{}({})]:\n{}\n", msg->name(), id, msg->message());
        fmt::format_to(std::back_inserter(buffer), "Geometries:\n");
        for(const auto& [name, geo] : msg->geometries())
        {
            fmt::format_to(std::back_inserter(buffer), "  - {}: <{}>\n", name, geo->type());
        }
        buffer.pop_back();  // remove the last '\n'
        spdlog::info(buffer);
        buffer.clear();
    }
}

const unordered_map<U64, S<SanityCheckMessage>>& SanityChecker::errors() const
{
    return m_errors.messages();
}
const unordered_map<U64, S<SanityCheckMessage>>& SanityChecker::warns() const
{
    return m_warns.messages();
}
const unordered_map<U64, S<SanityCheckMessage>>& SanityChecker::infos() const
{
    return m_infos.messages();
}
void SanityChecker::clear()
{
    m_errors.messages().clear();
    m_warns.messages().clear();
    m_infos.messages().clear();
}
}  // namespace uipc::core
