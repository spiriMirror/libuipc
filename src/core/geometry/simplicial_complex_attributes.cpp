#include <uipc/geometry/simplicial_complex_attributes.h>

namespace uipc::geometry
{
template class UIPC_CORE_API SimplicialComplexAttributes<false, 0>;
template class UIPC_CORE_API SimplicialComplexAttributes<true, 0>;

template class UIPC_CORE_API SimplicialComplexAttributes<false, 1>;
template class UIPC_CORE_API SimplicialComplexAttributes<true, 1>;

template class UIPC_CORE_API SimplicialComplexAttributes<false, 2>;
template class UIPC_CORE_API SimplicialComplexAttributes<true, 2>;

template class UIPC_CORE_API SimplicialComplexAttributes<false, 3>;
template class UIPC_CORE_API SimplicialComplexAttributes<true, 3>;
}  // namespace uipc::geometry
