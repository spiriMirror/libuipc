#pragma once
#include <uipc/common/macro.h>
#include <string>
#include <exception>

namespace uipc
{
class UIPC_CORE_API Exception : public std::exception
{
  public:
    //Exception(const std::string&   msg,
    //          std::source_location loc = std::source_location::current());

    Exception(const std::string& msg);

    virtual const char* what() const noexcept override;

    // std::string error_with_loc() const;

    // const std::source_location& loc() const;

  private:
    std::string m_msg;
    //std::source_location m_loc;
};
}  // namespace uipc
