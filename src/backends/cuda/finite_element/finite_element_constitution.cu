#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
U64 FiniteElementConstitution::uid() const noexcept
{
    return get_uid();
}

IndexT FiniteElementConstitution::dimension() const noexcept
{
    return get_dimension();
}

void FiniteElementConstitution::do_build(FiniteElementEnergyProducer::BuildInfo& info)
{
    m_finite_element_method = &require<FiniteElementMethod>();

    // Check if we have the FiniteElementConstitution
    auto uids = world().scene().constitution_tabular().uids();
    if(!std::binary_search(uids.begin(), uids.end(), uid()))
    {
        throw SimSystemException(fmt::format("Requires Constitution UID={}", uid()));
    }

    BuildInfo this_info;
    do_build(this_info);

    m_finite_element_method->add_constitution(this);
}

const FiniteElementMethod::DimInfo& FiniteElementConstitution::dim_info() const noexcept
{
    return fem().dim_infos[m_index_in_dim];
}

FiniteElementMethod::Impl& FiniteElementConstitution::fem() const noexcept
{
    return m_finite_element_method->m_impl;
}
}  // namespace uipc::backend::cuda
