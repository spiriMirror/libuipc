#pragma once
#include <uipc/builtin/uid_register.h>


namespace uipc::builtin
{
class UIPC_CORE_API ConstitutionUIDRegister : public details::UIDRegister
{
  public:
    static const ConstitutionUIDRegister& instance() noexcept;

  private:
    ConstitutionUIDRegister();
};
}  // namespace uipc::builtin
