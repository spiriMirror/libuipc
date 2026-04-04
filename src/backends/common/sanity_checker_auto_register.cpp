#include <backends/common/sanity_checker_auto_register.h>

namespace uipc::backend
{
SanityCheckerAutoRegister::SanityCheckerAutoRegister(Creator&& reg)
{
    creators().entries.push_back(std::move(reg));
}

auto SanityCheckerAutoRegister::creators() -> Creators&
{
    static Creators data;
    return data;
}
}  // namespace uipc::backend
