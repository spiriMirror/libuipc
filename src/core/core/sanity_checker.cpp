#include <uipc/core/sanity_checker.h>

namespace uipc::core
{
void ISanityChecker::build() {}

U64 ISanityChecker::id() const noexcept
{
    return get_id();
}

SanityCheckResult ISanityChecker::check()
{
    return do_check();
}
}  // namespace uipc::core
