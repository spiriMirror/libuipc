#include <uipc/world/sanity_check/sanity_checker_auto_register.h>

namespace uipc::world
{
SanityCheckerAutoRegister::SanityCheckerAutoRegister(Creator&& reg)
{
    creators().entries.push_back(std::move(reg));
}

auto SanityCheckerAutoRegister::creators() -> Creators&
{
    static Creators creators;
    return creators;
}
}  // namespace uipc::world
