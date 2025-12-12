#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ImplicitGeometryUIDCollection : public details::UIDRegister
{
  public:
    static const ImplicitGeometryUIDCollection& instance() noexcept;
  private:
    friend class ImplicitGeometryUIDAutoRegister;
    ImplicitGeometryUIDCollection();
};
}  // namespace uipc::builtin
