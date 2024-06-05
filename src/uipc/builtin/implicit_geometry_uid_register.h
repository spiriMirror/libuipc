#pragma once
#include <uipc/builtin/uid_register.h>


namespace uipc::builtin
{
class UIPC_CORE_API ImplicitGeometryUIDRegister : public details::UIDRegister
{
  public:
    static const ImplicitGeometryUIDRegister& instance() noexcept;

  private:
    ImplicitGeometryUIDRegister();
};
}  // namespace uipc::builtin
