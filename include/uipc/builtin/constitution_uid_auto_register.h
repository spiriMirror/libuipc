#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ConstitutionUIDAutoRegister
{
  public:
    using Creator = std::function<list<UIDInfo>()>;
    ConstitutionUIDAutoRegister(Creator creator) noexcept;

  private:
    friend class ConstitutionUIDCollection;
    static list<Creator>& creators() noexcept;
};
}  // namespace uipc::builtin

#define REGISTER_CONSTITUTION_UIDS_INTERNAL(counter)                                                  \
    namespace auto_register                                                                           \
    {                                                                                                 \
        static ::uipc::list<::uipc::builtin::UIDInfo> ConstitutionUIDAutoRegisterFunction##counter(); \
        static ::uipc::builtin::ConstitutionUIDAutoRegister ConstitutionUIDAutoRegister##counter{     \
            ConstitutionUIDAutoRegisterFunction##counter};                                            \
    }                                                                                                 \
    static ::uipc::list<::uipc::builtin::UIDInfo> auto_register::ConstitutionUIDAutoRegisterFunction##counter()

/**
 * @brief Register ConstitutionUIDs.
 * 
 * Example:
 * 
 * ```c++
 * ```
 */
#define REGISTER_CONSTITUTION_UIDS(...)                                        \
    REGISTER_CONSTITUTION_UIDS_INTERNAL(__COUNTER__)
