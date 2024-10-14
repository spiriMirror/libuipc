#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
U64 FiniteElementConstitution::uid() const
{
    return get_uid();
}

IndexT FiniteElementConstitution::dimension() const
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

    BuildInfo info;
    do_build(info);

    m_finite_element_method->add_constitution(this);
}

void FiniteElementConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto primitive_count = dim_info().primitive_count;
    info.energy_count(primitive_count);
    // Codim0D has 1 vertex, Codim1D has 2 vertices, Codim2D has 3 vertices, FEM3D has 4 vertices
    info.stencil_dim(dimension() + 1);
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
