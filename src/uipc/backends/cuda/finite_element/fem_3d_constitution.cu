#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
void FEM3DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    FEM3DConstitution::BuildInfo this_info;
    do_build(this_info);
}

IndexT FEM3DConstitution::get_dimension() const
{
    return 3;
}
}  // namespace uipc::backend::cuda
