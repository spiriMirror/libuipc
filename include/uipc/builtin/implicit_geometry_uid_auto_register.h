#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ImplicitGeometryUIDAutoRegister
{
  public:
    using Creator = std::function<list<UIDInfo>()>;
    ImplicitGeometryUIDAutoRegister(Creator creator) noexcept;

  private:
    friend class ImplicitGeometryUIDCollection;
    static list<Creator>& creators() noexcept;
};
}  // namespace uipc::builtin

#define REGISTER_IMPLICIT_GEOMETRY_UIDS_INTERNAL(counter)                                                 \
    namespace auto_register                                                                               \
    {                                                                                                     \
        static ::uipc::list<::uipc::builtin::UIDInfo> ImplicitGeometryUIDAutoRegisterFunction##counter(); \
        static ::uipc::builtin::ImplicitGeometryUIDAutoRegister ImplicitGeometryUIDAutoRegister##counter{ \
            ImplicitGeometryUIDAutoRegisterFunction##counter};                                            \
    }                                                                                                     \
    static ::uipc::list<::uipc::builtin::UIDInfo> auto_register::ImplicitGeometryUIDAutoRegisterFunction##counter()

/**
 * @brief Register ImplicitGeometryUIDs.
 */
#define REGISTER_IMPLICIT_GEOMETRY_UIDS(...)                                   \
    REGISTER_IMPLICIT_GEOMETRY_UIDS_INTERNAL(__COUNTER__)

//