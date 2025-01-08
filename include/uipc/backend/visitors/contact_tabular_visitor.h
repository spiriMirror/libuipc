#pragma once
#include <uipc/geometry/attribute_collection.h>

namespace uipc::core
{
class ContactTabular;
}

namespace uipc::backend
{
class UIPC_CORE_API ContactTabularVisitor
{
  public:
    ContactTabularVisitor(core::ContactTabular& contact_tabular) noexcept
        : m_contact_tabular(contact_tabular)
    {
    }

    geometry::AttributeCollection& contact_models() noexcept;

  private:
    core::ContactTabular& m_contact_tabular;
};
}  // namespace uipc::backend
