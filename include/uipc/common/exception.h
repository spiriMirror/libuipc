#pragma once
#include <uipc/common/string.h>
#include <uipc/common/dllexport.h>
#include <exception>
#include <cpptrace/cpptrace.hpp>

namespace uipc
{
class UIPC_CORE_API Exception : public cpptrace::exception_with_message
{
  public:
    Exception(std::string_view msg);
};
}  // namespace uipc
