#pragma once
#include <uipc/geometry/attribute_collection.h>

namespace uipc::core
{
class SubsceneTabular;
}

namespace uipc::backend
{
class UIPC_CORE_API SubsceneTabularVisitor
{
  public:
    SubsceneTabularVisitor(core::SubsceneTabular& subscene_tabular) noexcept
        : m_subscene_tabular(subscene_tabular)
    {
    }

    geometry::AttributeCollection& subscene_models() noexcept;

  private:
    core::SubsceneTabular& m_subscene_tabular;
};
}  // namespace uipc::backend
