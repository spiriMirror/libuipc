#include <uipc/world/sanity_check/i_sanity_checker.h>

namespace uipc::world
{
U64 ISanityChecker::id() const noexcept
{
    return get_id();
}

SanityCheckResult ISanityChecker::check() noexcept
{
    return do_check();
}
}  // namespace uipc::world
