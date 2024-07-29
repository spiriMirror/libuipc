#include <finite_element/codim_1d_constitution.h>

namespace uipc::backend::cuda
{
void Codim1DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim1DConstitution::BuildInfo this_info;
    do_build(this_info);
}

IndexT Codim1DConstitution::get_dimension() const
{
    return 1;
}
}  // namespace uipc::backend::cuda
