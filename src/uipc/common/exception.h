#pragma once
#include <uipc/common/macro.h>
#include <string>
#include <exception>

namespace uipc
{
class UIPC_CORE_API Exception : public std::exception
{
  public:
    Exception(const std::string& msg);

    virtual const char* what() const noexcept override;

  private:
    std::string m_msg;
};
}  // namespace uipc
