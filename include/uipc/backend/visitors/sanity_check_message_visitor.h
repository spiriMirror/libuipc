#pragma once
#include <uipc/common/type_define.h>
#include <uipc/core/i_sanity_checker.h>

namespace uipc::backend
{
class UIPC_CORE_API SanityCheckMessageVisitor
{
  public:
    SanityCheckMessageVisitor(core::SanityCheckMessage& msg) noexcept;

    auto& id() const noexcept { return m_msg.m_id; }
    auto& name() const noexcept { return m_msg.m_name; }
    auto& result() const noexcept { return m_msg.m_result; }
    auto& message() const noexcept { return m_msg.m_message; }
    auto& geometries() const noexcept { return m_msg.m_geometries; }

  private:
    core::SanityCheckMessage& m_msg;
};
}  // namespace uipc::backend
