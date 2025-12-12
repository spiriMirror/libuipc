#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ImplicitGeometryUIDAutoRegister
{
  public:
    using Creator = UIDInfoCreator;

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
            ::uipc::builtin::ImplicitGeometryUIDAutoRegister::Creator{                                    \
                ImplicitGeometryUIDAutoRegisterFunction##counter, __FILE__, __LINE__}};                   \
    }                                                                                                     \
    static ::uipc::list<::uipc::builtin::UIDInfo> auto_register::ImplicitGeometryUIDAutoRegisterFunction##counter()

/**
 * @brief Register ImplicitGeometryUIDs.
 * 
 * Example:
 * 
 * ```c++
 *  REGISTER_IMPLICIT_GEOMETRY_UIDS()
 *  {
 *      using namespace uipc::builtin;
 *      list<UIDInfo> uids;
 *      return uids;
 *  }
 * ```
 */
#define REGISTER_IMPLICIT_GEOMETRY_UIDS(...)                                   \
    REGISTER_IMPLICIT_GEOMETRY_UIDS_INTERNAL(__COUNTER__)