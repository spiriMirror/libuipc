#include <uipc/common/exception.h>
#include <uipc/common/format.h>

namespace uipc
{
Exception::Exception(const std::string& msg)
    : m_msg{msg}
{
}

const char* Exception::what() const noexcept
{
    return m_msg.c_str();
}
}  // namespace uipc
