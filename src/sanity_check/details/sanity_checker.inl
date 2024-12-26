#include <sanity_checker_collection.h>

namespace uipc::sanity_check
{
template <std::derived_from<core::ISanityChecker> SanityCheckerT>
SanityCheckerT* SanityChecker::find() const
{
    return m_collection.find<SanityCheckerT>();
}

template <std::derived_from<core::ISanityChecker> SanityCheckerT>
SanityCheckerT& SanityChecker::require() const
{
    return m_collection.require<SanityCheckerT>();
}
}  // namespace uipc::sanity_check
