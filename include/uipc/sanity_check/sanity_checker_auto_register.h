#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/common/type_traits.h>
#include <uipc/sanity_check/i_sanity_checker.h>
namespace uipc::core
{
class UIPC_CORE_API SanityCheckerAutoRegister
{
    friend class Scene;
    friend class SanityCheckerCollection;

  public:
    using Creator = std::function<U<ISanityChecker>(const Scene&)>;

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
    template <std::derived_from<ISanityChecker> SanityCheckerT>
    std::function<U<ISanityChecker>(const Scene&)> register_sanity_checker_creator()
    {
        return [](const Scene& scene) -> U<ISanityChecker>
        {
            return ::uipc::static_pointer_cast<ISanityChecker>(
                ::uipc::make_unique<SanityCheckerT>(scene));
        };
    }
}  // namespace detail
}  // namespace uipc::core
