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
void FiniteElementConstitution::do_build()
{
    auto& finite_element_method = require<FiniteElementMethod>();
    finite_element_method.add_constitution(this);
}
}  // namespace uipc::backend::cuda
