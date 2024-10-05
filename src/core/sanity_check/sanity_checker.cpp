#include <uipc/sanity_check/sanity_checker.h>

namespace uipc::core
{
SanityChecker::SanityChecker(const Scene& s) noexcept
    : m_scene(s)
{
}

SanityCheckResult SanityChecker::do_check() noexcept
{
    return do_check(m_scene.m_impl);
}
}  // namespace uipc::core
