#include <uipc/usd/prim.h>
#include <private/prim.h>
#include <uipc/common/log.h>

namespace uipc::usd
{
S<geometry::Geometry> Prim::to_geometry()
{
    UIPC_ASSERT(false, "NOT IMPLEMENTED");
    return nullptr;
}

void Prim::from_geometry(geometry::Geometry& geo)
{
    UIPC_ASSERT(false, "NOT IMPLEMENTED");
}

Prim::Prim(S<Impl> impl):
    m_impl{std::move(impl)}
{
}
}  // namespace uipc::usd
