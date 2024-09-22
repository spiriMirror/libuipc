#pragma once
#include <uipc/builtin/uid_register.h>
#include <uipc/common/list.h>

namespace uipc::builtin
{
class UIPC_CORE_API ConstitutionUIDCollection : public details::UIDRegister
{
  public:
    static const ConstitutionUIDCollection& instance() noexcept;
  private:
    friend class ConstitutionUIDAutoRegister;
    ConstitutionUIDCollection();
};
}  // namespace uipc::builtin