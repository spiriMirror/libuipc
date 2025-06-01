#include "sanity_checker.h"
#include <uipc/common/demangle.h>
#include <sanity_checker_collection.h>

namespace uipc::sanity_check
{
template <std::derived_from<core::ISanityChecker> SanityCheckerT>
inline SanityCheckerT* SanityCheckerCollection::find() const
{
    for(const auto& entry : m_entries)
    {
        if(auto* p = dynamic_cast<SanityCheckerT*>(entry.get()))
        {
            return p;
        }
    }
    return nullptr;
}

template <std::derived_from<core::ISanityChecker> SanityCheckerT>
inline SanityCheckerT& SanityCheckerCollection::require() const
{
    for(const auto& entry : m_entries)
    {
        if(auto* p = dynamic_cast<SanityCheckerT*>(entry.get()))
        {
            return *p;
        }
    }

    std::string name = uipc::demangle<SanityCheckerT>();

    throw SanityCheckerException{fmt::format("SanityChecker[{}] not found", name)};
}
}  // namespace uipc::sanity_check