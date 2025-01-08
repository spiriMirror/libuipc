#include <uipc/backend/visitors/contact_tabular_visitor.h>
#include <uipc/core/contact_tabular.h>

namespace uipc::backend
{
geometry::AttributeCollection& ContactTabularVisitor::contact_models() noexcept
{
    return m_contact_tabular.internal_contact_models();
}
}  // namespace uipc::backend
