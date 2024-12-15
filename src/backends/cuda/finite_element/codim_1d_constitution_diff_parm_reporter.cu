#include <finite_element/codim_1d_constitution_diff_parm_reporter.h>

namespace uipc::backend::cuda
{
void Codim1DConstitutionDiffParmReporter::do_assemble(Base::DiffParmInfo& info)
{
    DiffParmInfo this_info(this, info);
    do_assemble(this_info);
}

IndexT Codim1DConstitutionDiffParmReporter::get_dim() const noexcept
{
    return 1;
}

muda::CBufferView<Vector3> Codim1DConstitutionDiffParmReporter::DiffParmInfo::xs() const noexcept
{
    return m_impl->fem().xs.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Vector3> Codim1DConstitutionDiffParmReporter::DiffParmInfo::x_bars() const noexcept
{
    return m_impl->fem().x_bars.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Float> Codim1DConstitutionDiffParmReporter::DiffParmInfo::rest_lengths() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().rest_lengths.view(info.primitive_offset, info.primitive_count);
}

muda::CBufferView<Float> Codim1DConstitutionDiffParmReporter::DiffParmInfo::thicknesses() const noexcept
{
    return m_impl->fem().thicknesses.view();
}

muda::CBufferView<IndexT> Codim1DConstitutionDiffParmReporter::DiffParmInfo::is_fixed() const noexcept
{
    return m_impl->fem().is_fixed.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Vector2i> Codim1DConstitutionDiffParmReporter::DiffParmInfo::indices() const noexcept
{
    auto& info = constitution_info();
    return m_impl->fem().codim_1ds.view(info.primitive_offset, info.primitive_count);
}

const FiniteElementMethod::ConstitutionInfo& Codim1DConstitutionDiffParmReporter::DiffParmInfo::constitution_info() const noexcept
{
    return m_impl->constitution_info();
}

SizeT Codim1DConstitutionDiffParmReporter::DiffParmInfo::frame() const
{
    return m_diff_parm_info.frame();
}
IndexT Codim1DConstitutionDiffParmReporter::DiffParmInfo::dof_offset(SizeT frame) const
{
    return m_diff_parm_info.dof_offset(frame);
}
IndexT Codim1DConstitutionDiffParmReporter::DiffParmInfo::dof_count(SizeT frame) const
{
    return m_diff_parm_info.dof_count(frame);
}

muda::TripletMatrixView<Float, 1> Codim1DConstitutionDiffParmReporter::DiffParmInfo::pGpP() const
{
    return m_diff_parm_info.pGpP();
}

Float Codim1DConstitutionDiffParmReporter::DiffParmInfo::dt() const
{
    return m_diff_parm_info.dt();
}
}  // namespace uipc::backend::cuda
