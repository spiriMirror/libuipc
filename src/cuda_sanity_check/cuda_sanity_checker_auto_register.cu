#include <cuda_sanity_checker_auto_register.h>

namespace uipc::cuda_sanity_check
{
CudaSanityCheckerAutoRegister::CudaSanityCheckerAutoRegister(Creator&& reg)
{
    creators().entries.push_back(std::move(reg));
}

auto CudaSanityCheckerAutoRegister::creators() -> Creators&
{
    static Creators creators;
    return creators;
}
}  // namespace uipc::cuda_sanity_check
