#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/smart_pointer.h>

namespace uipc
{
class UIPC_CORE_API ResidentThread
{
  public:
    ResidentThread();

    SizeT hash() const;

    ~ResidentThread();

    bool post(std::function<void()> task);
    bool is_ready() const;

    class Impl;

  private:
    S<Impl> m_impl;
};
}  // namespace uipc