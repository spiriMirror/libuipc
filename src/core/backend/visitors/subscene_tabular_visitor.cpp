#include <uipc/backend/visitors/subscene_tabular_visitor.h>
#include <uipc/core/subscene_tabular.h>

namespace uipc::backend
{
geometry::AttributeCollection& SubsceneTabularVisitor::subscene_models() noexcept
{
    return m_subscene_tabular.internal_subscene_models();
}
}  // namespace uipc::backend
