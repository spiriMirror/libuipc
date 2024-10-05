#include <uipc/sanity_check/i_sanity_checker.h>

namespace uipc::core
{
U64 ISanityChecker::id() const noexcept
{
    return get_id();
}

SanityCheckResult ISanityChecker::check() noexcept
{
    return do_check();
}
}  // namespace uipc::core
