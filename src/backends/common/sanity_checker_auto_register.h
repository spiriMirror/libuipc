#pragma once
#include <functional>
#include <uipc/common/smart_pointer.h>
#include <uipc/common/list.h>
#include <uipc/common/macro.h>
#include <uipc/core/i_sanity_checker.h>

namespace uipc::backend
{
class SanityCheckerAutoRegister
{
  public:
    using Creator =
        std::function<S<core::ISanityChecker>(core::ISanityCheckContext&)>;

    SanityCheckerAutoRegister(Creator&& reg);

    class Creators
    {
      public:
        list<Creator> entries;
    };

    static Creators& creators();
};
}  // namespace uipc::backend

#define REGISTER_BACKEND_SANITY_CHECKER(Type)                                                               \
    namespace auto_register                                                                                 \
    {                                                                                                       \
        static ::uipc::backend::SanityCheckerAutoRegister UIPC_NAME_WITH_ID(SanityCheckerAutoRegister){     \
            [](::uipc::core::ISanityCheckContext& ctx) -> ::uipc::S<::uipc::core::ISanityChecker> {        \
                return ::uipc::make_shared<Type>(ctx);                                                      \
            }};                                                                                             \
    }
