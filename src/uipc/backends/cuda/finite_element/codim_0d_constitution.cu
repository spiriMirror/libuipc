#include <finite_element/codim_0d_constitution.h>

namespace uipc::backend::cuda
{
void Codim0DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim0DConstitution::BuildInfo this_info;
    do_build(this_info);
}

IndexT Codim0DConstitution::get_dimension() const
{
    return 0;
}
}  // namespace uipc::backend::cuda
