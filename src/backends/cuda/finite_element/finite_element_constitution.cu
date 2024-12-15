#include <finite_element/finite_element_constitution.h>

namespace uipc::backend::cuda
{
U64 FiniteElementConstitution::uid() const noexcept
{
    return get_uid();
}

IndexT FiniteElementConstitution::dim() const noexcept
{
    return get_dim();
}

void FiniteElementConstitution::init()
{
    FiniteElementMethod::FilteredInfo info{&fem(), dim(), m_index_in_dim};
    do_init(info);
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

void FiniteElementConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto& c_info = constitution_info();
    info.energy_count(c_info.primitive_count);
    info.stencil_dim(dim() + 1);
}

const FiniteElementMethod::DimInfo& FiniteElementConstitution::dim_info() const noexcept
{
    return fem().dim_infos[m_index_in_dim];
}

const FiniteElementMethod::ConstitutionInfo& FiniteElementConstitution::constitution_info() const noexcept
{
    switch(dim())
    {
        case 0:
            return fem().codim_0d_constitution_infos[m_index_in_dim];
        case 1:
            return fem().codim_1d_constitution_infos[m_index_in_dim];
        case 2:
            return fem().codim_2d_constitution_infos[m_index_in_dim];
        case 3:
            return fem().fem_3d_constitution_infos[m_index_in_dim];
        default:
            UIPC_ASSERT(false, "Invalid Dim {}D", dim());
            break;
    }
}

FiniteElementMethod::Impl& FiniteElementConstitution::fem() const noexcept
{
    return m_finite_element_method->m_impl;
}
}  // namespace uipc::backend::cuda
