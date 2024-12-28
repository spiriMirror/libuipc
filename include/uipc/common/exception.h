#pragma once
#include <uipc/common/string.h>
#include <uipc/common/dllexport.h>
#include <exception>

namespace uipc
{
class UIPC_CORE_API Exception : public std::exception
{
  public:
    Exception(const string& msg);

    virtual const char* what() const noexcept override;

  private:
    string m_msg;
};
}  // namespace uipc
