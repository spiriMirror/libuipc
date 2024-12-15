#include <uipc/sanity_check/sanity_checker_collection.h>
#include <uipc/sanity_check/sanity_checker_auto_register.h>

namespace uipc::core
{
void SanityCheckerCollection::init(Scene& s)
{
    for(const auto& creator : SanityCheckerAutoRegister::creators().entries)
    {
        entries.push_back(creator(s));
    }
}

SanityCheckResult SanityCheckerCollection::check() const
{
    int result = static_cast<int>(SanityCheckResult::Success);
    for(const auto& entry : entries)
    {
        int check = static_cast<int>(entry->check());

        if(check > result)
            result = check;
    }
    return static_cast<SanityCheckResult>(result);
}
}  // namespace uipc::core
