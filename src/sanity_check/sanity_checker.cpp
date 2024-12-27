#include <sanity_checker.h>
#include <sanity_checker_collection.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>
#include <boost/core/demangle.hpp>

namespace uipc::sanity_check
{
SanityChecker::SanityChecker(SanityCheckerCollection& c, core::Scene& s) noexcept
    : m_collection{c}
    , m_scene{s}
{
}

std::string_view SanityChecker::workspace() const noexcept
{
    return m_collection.workspace();
}

std::string SanityChecker::this_output_path() const noexcept
{
    namespace fs = std::filesystem;

    fs::path path{workspace()};
    path /= fmt::format("{}", get_id());
    std::filesystem::exists(path) || std::filesystem::create_directories(path);
    return path.string();
}

std::string SanityChecker::get_name() const noexcept
{
    return boost::core::demangle(typeid(*this).name());
}

void SanityChecker::build(backend::SceneVisitor& scene) {}

core::Scene::CObjects SanityChecker::objects() const noexcept
{
    return std::as_const(m_scene).objects();
}

void SanityChecker::build()
{
    backend::SceneVisitor sv{m_scene};
    build(sv);
}

SanityCheckResult SanityChecker::do_check(core::SanityCheckMessage& msg)
{
    backend::SceneVisitor              sv{m_scene};
    backend::SanityCheckMessageVisitor scmv{msg};

    scmv.id()     = get_id();
    scmv.name()   = get_name();
    scmv.result() = SanityCheckResult::Success;

    auto result   = do_check(sv, scmv);
    scmv.result() = result;

    return result;
}
}  // namespace uipc::sanity_check
