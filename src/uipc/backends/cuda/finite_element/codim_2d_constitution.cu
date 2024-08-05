#include <finite_element/codim_2d_constitution.h>

namespace uipc::backend::cuda
{
void Codim2DConstitution::retrieve(FiniteElementMethod::Codim2DFilteredInfo& info)
{
    UIPC_ASSERT(false, "Not implemented");
}

void Codim2DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim2DConstitution::BuildInfo this_info;
    do_build(this_info);
}

IndexT Codim2DConstitution::get_dimension() const
{
    return 2;
}
}  // namespace uipc::backend::cuda
