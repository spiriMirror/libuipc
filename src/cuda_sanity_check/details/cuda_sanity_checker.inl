#include <cuda_sanity_checker_collection.h>

namespace uipc::cuda_sanity_check
{
template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
CudaSanityCheckerT* CudaSanityChecker::find() const
{
    return m_collection.find<CudaSanityCheckerT>();
}

template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
CudaSanityCheckerT& CudaSanityChecker::require() const
{
    return m_collection.require<CudaSanityCheckerT>();
}
}  // namespace uipc::cuda_sanity_check
