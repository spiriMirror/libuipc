#include <uipc/backend/visitors/sanity_check_message_visitor.h>

namespace uipc::backend
{
SanityCheckMessageVisitor::SanityCheckMessageVisitor(core::SanityCheckMessage& msg) noexcept
    : m_msg{msg}
{
}
}  // namespace uipc::backend
