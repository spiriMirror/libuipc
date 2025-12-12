#pragma once
#include <uipc/builtin/uid_register.h>

namespace uipc::builtin
{
class UIPC_CORE_API ImplicitGeometryUIDCollection : public details::UIDRegister
{
  public:
    static const ImplicitGeometryUIDCollection& instance() noexcept;

  private:
    ImplicitGeometryUIDCollection();
};
}  // namespace uipc::builtin
