#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/common/type_traits.h>
#include <uipc/core/sanity_checker.h>

namespace uipc::sanity_check
{
class SanityCheckerCollection;
class SanityCheckerAutoRegister
{
    friend class Scene;
    friend class SanityCheckerCollection;

  public:
    using Creator =
        std::function<U<core::ISanityChecker>(SanityCheckerCollection&, core::internal::Scene&)>;

    SanityCheckerAutoRegister(Creator&& reg);

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
    template <std::derived_from<core::ISanityChecker> SanityCheckerT>
    SanityCheckerAutoRegister::Creator register_sanity_checker_creator()
    {
        return [](SanityCheckerCollection& c, core::internal::Scene& scene) -> U<core::ISanityChecker>
        {
            return ::uipc::static_pointer_cast<core::ISanityChecker>(
                ::uipc::make_unique<SanityCheckerT>(c, scene));
        };
    }
}  // namespace detail
}  // namespace uipc::sanity_check
