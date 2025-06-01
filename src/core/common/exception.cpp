#include <uipc/common/exception.h>
#include <uipc/common/format.h>

namespace uipc
{
Exception::Exception(std::string_view msg)
    : cpptrace::exception_with_message{std::string{msg}}
{
}
}  // namespace uipc
