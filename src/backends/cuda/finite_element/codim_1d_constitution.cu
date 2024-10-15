#include <finite_element/codim_1d_constitution.h>

namespace uipc::backend::cuda
{
void Codim1DConstitution::init(FiniteElementMethod::Codim1DFilteredInfo& info)
{
    do_init(info);
}

void Codim1DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim1DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void Codim1DConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto& c_info = constitution_info();
    info.energy_count(c_info.primitive_count);
    info.stencil_dim(dimension() + 1);
}

void Codim1DConstitution::do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info)
{
    Codim1DConstitution::ComputeEnergyInfo this_info{
        &fem(), m_index_in_dim, info.dt(), info.energies()};
    do_compute_energy(this_info);
}

void Codim1DConstitution::do_compute_gradient_hessian(FiniteElementEnergyProducer::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{
        &fem(), m_index_in_dim, info.dt(), info.gradients(), info.hessians()};
    do_compute_gradient_hessian(this_info);
}

const FiniteElementMethod::ConstitutionInfo& Codim1DConstitution::constitution_info() const noexcept
{
    return fem().codim_1d_constitution_infos[m_index_in_dim];
}

IndexT Codim1DConstitution::get_dimension() const noexcept
{
    return 1;
}

Vector2i Codim1DConstitution::get_vertex_offset_count() const noexcept
{
    auto& info = constitution_info();
    return Vector2i{info.vertex_offset, info.vertex_count};
}

muda::CBufferView<Vector3> Codim1DConstitution::BaseInfo::xs() const noexcept
{
    return m_fem->xs.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Vector3> Codim1DConstitution::BaseInfo::x_bars() const noexcept
{
    return m_fem->x_bars.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Float> Codim1DConstitution::BaseInfo::rest_lengths() const noexcept
{
    auto& info = constitution_info();
    return m_fem->rest_lengths.view(info.primitive_offset, info.primitive_count);
}

muda::CBufferView<Float> Codim1DConstitution::BaseInfo::thicknesses() const noexcept
{
    return m_fem->thicknesses.view();
}

muda::CBufferView<Vector2i> Codim1DConstitution::BaseInfo::indices() const noexcept
{
    auto& info = constitution_info();
    return m_fem->codim_1ds.view(info.primitive_offset, info.primitive_count);
}

const FiniteElementMethod::ConstitutionInfo& Codim1DConstitution::BaseInfo::constitution_info() const noexcept
{
    return m_fem->codim_1d_constitution_infos[m_index_in_dim];
}

Float Codim1DConstitution::BaseInfo::dt() const noexcept
{
    return m_dt;
}
}  // namespace uipc::backend::cuda
