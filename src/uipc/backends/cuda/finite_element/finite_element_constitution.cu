#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
void FiniteElementConstitution::do_build()
{
    m_fem = &require<FiniteElementMethod>();

    // Check if we have the Affine Body Constitution
    auto uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(uids.begin(), uids.end(), constitution_uid()))
    {
        throw SimSystemException(
            fmt::format("Requires Constitution UID={}", constitution_uid()));
    }

    BuildInfo info;
    do_build(info);

    m_fem->add_constitution(this);
}

void FiniteElementConstitution::compute_energy(FiniteElementMethod::ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void FiniteElementConstitution::compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
}

U64 FiniteElementConstitution::constitution_uid() const
{
    return get_constitution_uid();
}

IndexT FiniteElementConstitution::dimension() const
{
    return get_dimension();
}
}  // namespace uipc::backend::cuda
