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

void FiniteElementConstitution::do_build()
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

void FiniteElementConstitution::report_extent(ReportExtentInfo& info)
{
    do_report_extent(info);
    info.check(name());

    UIPC_ASSERT(!(info.gradient_only() && info.m_hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name(),
                info.m_hessian_count);
}

void FiniteElementConstitution::compute_energy(ComputeEnergyInfo& info)
{
    do_compute_energy(info);
}

void FiniteElementConstitution::compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    do_compute_gradient_hessian(info);
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
            // dummy return to suppress compiler warning
            return fem().fem_3d_constitution_infos[m_index_in_dim];
    }
}

FiniteElementMethod::Impl& FiniteElementConstitution::fem() const noexcept
{
    return m_finite_element_method->m_impl;
}
}  // namespace uipc::backend::cuda
