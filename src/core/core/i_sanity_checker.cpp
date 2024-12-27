#include <uipc/core/i_sanity_checker.h>

namespace uipc::core
{
void ISanityChecker::build() {}

U64 ISanityChecker::id() const noexcept
{
    return get_id();
}

SanityCheckResult ISanityChecker::check(SanityCheckMessage& msg)
{
    return do_check(msg);
}

std::string ISanityChecker::name() const noexcept
{
    return get_name();
}

U64 SanityCheckMessage::id() const noexcept
{
    return m_id;
}

std::string_view SanityCheckMessage::name() const noexcept
{
    return m_name;
}

SanityCheckResult SanityCheckMessage::result() const noexcept
{
    return m_result;
}

std::string_view SanityCheckMessage::message() const noexcept
{
    return m_message;
}

const unordered_map<std::string, S<geometry::Geometry>>& SanityCheckMessage::geometries() const noexcept
{
    return m_geometries;
}

bool SanityCheckMessage::is_empty() const noexcept
{
    return m_result == SanityCheckResult::Success && m_message.empty()
           && m_geometries.empty();
}
}  // namespace uipc::core
