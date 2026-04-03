#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/common/type_traits.h>
#include <uipc/core/i_sanity_checker.h>

namespace uipc::cuda_sanity_check
{
class CudaSanityCheckerCollection;

class CudaSanityCheckerAutoRegister
{
    friend class CudaSanityCheckerCollection;

  public:
    using Creator =
        std::function<U<core::ISanityChecker>(CudaSanityCheckerCollection&, core::internal::Scene&)>;

    CudaSanityCheckerAutoRegister(Creator&& reg);

    class Creators
    {
      public:
        list<Creator> entries;
    };

  private:
    static Creators& creators();
};

namespace detail
{
    template <std::derived_from<core::ISanityChecker> CudaSanityCheckerT>
    CudaSanityCheckerAutoRegister::Creator register_cuda_sanity_checker_creator()
    {
        return [](CudaSanityCheckerCollection& c, core::internal::Scene& scene) -> U<core::ISanityChecker>
        {
            return ::uipc::static_pointer_cast<core::ISanityChecker>(
                ::uipc::make_unique<CudaSanityCheckerT>(c, scene));
        };
    }
}  // namespace detail
}  // namespace uipc::cuda_sanity_check
