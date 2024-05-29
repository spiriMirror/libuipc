#include <uipc/common/exception.h>
#include <uipc/common/format.h>

namespace uipc
{
Exception::Exception(const std::string& msg, std::source_location loc)
    : m_msg{msg}
    , m_loc{loc}
{
}

const char* Exception::what() const noexcept
{
    return m_msg.c_str();
}

std::string Exception::error_with_loc() const
{
    return fmt::format("{}:[{}({})]", m_msg, m_loc.file_name(), m_loc.line());
}

const std::source_location& Exception::loc() const
{
    return m_loc;
}
}  // namespace uipc
