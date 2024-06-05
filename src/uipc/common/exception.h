#pragma once
#include <uipc/common/type_define.h>
#include <uipc/common/macro.h>
#include <string>
#include <exception>
#include <source_location>
namespace uipc
{
class UIPC_CORE_API Exception : public std::exception
{
  public:
    Exception(const std::string&   msg,
              std::source_location loc = std::source_location::current());

    virtual const char* what() const noexcept override;

    std::string error_with_loc() const;

    const std::source_location& loc() const;

  private:
    std::string          m_msg;
    std::source_location m_loc;
};
}  // namespace uipc
