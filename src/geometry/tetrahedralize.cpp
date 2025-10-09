#include <uipc/geometry/utils/tetrahedralize.h>
#include <uipc/geometry/utils/closure.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::geometry
{
SimplicialComplex tetrahedralize(const SimplicialComplex& sc, const Json& options)
{
    UIPC_ASSERT(false, "tetgen is removed!");
    return SimplicialComplex{};
}
}  // namespace uipc::geometry
