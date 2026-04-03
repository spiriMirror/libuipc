#include "cuda_sanity_checker.h"
#include <uipc/common/demangle.h>
#include <cuda_sanity_checker_collection.h>

namespace uipc::cuda_sanity_check
{
template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
inline CudaSanityCheckerT* CudaSanityCheckerCollection::find() const
{
    for(const auto& entry : m_entries)
    {
        if(auto* p = dynamic_cast<CudaSanityCheckerT*>(entry.get()))
        {
            return p;
        }
    }
    return nullptr;
}

template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
inline CudaSanityCheckerT& CudaSanityCheckerCollection::require() const
{
    for(const auto& entry : m_entries)
    {
        if(auto* p = dynamic_cast<CudaSanityCheckerT*>(entry.get()))
        {
            return *p;
        }
    }

    std::string name = uipc::demangle<CudaSanityCheckerT>();

    throw CudaSanityCheckerException{fmt::format("CudaSanityChecker[{}] not found", name)};
}
}  // namespace uipc::cuda_sanity_check
