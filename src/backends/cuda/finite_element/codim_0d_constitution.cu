#include <finite_element/codim_0d_constitution.h>

namespace uipc::backend::cuda
{
void Codim0DConstitution::init(FiniteElementMethod::Codim0DFilteredInfo& info)
{
    do_init(info);
}

void Codim0DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim0DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void Codim0DConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto& c_info = constitution_info();
    info.energy_count(0);  // not need to create any elastic energy
    info.stencil_dim(dimension() + 1);
}

const FiniteElementMethod::ConstitutionInfo& Codim0DConstitution::constitution_info() const noexcept
{
    return fem().codim_0d_constitution_infos[m_index_in_dim];
}

IndexT Codim0DConstitution::get_dimension() const noexcept
{
    return 0;
}

void Codim0DConstitution::do_compute_energy(FiniteElementConstitution::ComputeEnergyInfo& info)
{
    // do nothing
}

void Codim0DConstitution::do_compute_gradient_hessian(FiniteElementConstitution::ComputeGradientHessianInfo& info)
{
    // do nothing
}
Vector2i Codim0DConstitution::get_vertex_offset_count() const noexcept
{
    auto& info = constitution_info();
    return Vector2i{info.vertex_offset, info.vertex_count};
}
}  // namespace uipc::backend::cuda
