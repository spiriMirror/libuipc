#pragma once
#include <uipc/builtin/uid_register.h>

namespace uipc::builtin
{
class UIPC_CORE_API ConstitutionUIDCollection : public details::UIDRegister
{
  public:
    static const ConstitutionUIDCollection& instance() noexcept;

  private:
    ConstitutionUIDCollection();
};
}  // namespace uipc::builtin