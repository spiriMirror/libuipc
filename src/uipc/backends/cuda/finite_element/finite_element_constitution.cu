#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
U64 FiniteElementConstitution::constitution_uid() const
{
    return get_constitution_uid();
}
IndexT FiniteElementConstitution::dimension() const
{
    return get_dimension();
}
}  // namespace uipc::backend::cuda
