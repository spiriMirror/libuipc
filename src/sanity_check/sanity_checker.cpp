#include <sanity_checker.h>
#include <sanity_checker_collection.h>
#include <uipc/backend/visitors/scene_visitor.h>

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

SanityCheckResult SanityChecker::do_check()
{
    backend::SceneVisitor sv{m_scene};
    return do_check(sv);
}
}  // namespace uipc::sanity_check
