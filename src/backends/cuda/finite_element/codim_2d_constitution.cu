#include <finite_element/codim_2d_constitution.h>

namespace uipc::backend::cuda
{
void Codim2DConstitution::init(FiniteElementMethod::Codim2DFilteredInfo& info)
{
    do_init(info);
}

void Codim2DConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto& c_info = constitution_info();
    info.energy_count(c_info.primitive_count);
    info.stencil_dim(dimension() + 1);
}

void Codim2DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim2DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void Codim2DConstitution::do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info)
{
    Codim2DConstitution::ComputeEnergyInfo this_info{
        &fem(), m_index_in_dim, info.dt(), info.energies()};
    do_compute_energy(this_info);
}

void Codim2DConstitution::do_compute_gradient_hessian(FiniteElementEnergyProducer::ComputeGradientHessianInfo& info)
{
    Codim2DConstitution::ComputeGradientHessianInfo this_info{
        &fem(), m_index_in_dim, info.dt(), info.gradients(), info.hessians()};
    do_compute_gradient_hessian(this_info);
}

const FiniteElementMethod::ConstitutionInfo& Codim2DConstitution::constitution_info() const noexcept
{
    return fem().codim_2d_constitution_infos[m_index_in_dim];
}

IndexT Codim2DConstitution::get_dimension() const noexcept
{
    return 2;
}

Vector2i Codim2DConstitution::get_vertex_offset_count() const noexcept
{
    auto& info = constitution_info();
    return Vector2i{info.vertex_offset, info.vertex_count};
}

muda::CBufferView<Vector3> Codim2DConstitution::BaseInfo::xs() const noexcept
{
    return m_fem->xs.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Vector3> Codim2DConstitution::BaseInfo::x_bars() const noexcept
{
    return m_fem->x_bars.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Float> Codim2DConstitution::BaseInfo::rest_areas() const noexcept
{
    auto& info = constitution_info();
    return m_fem->rest_areas.view(info.primitive_offset, info.primitive_count);
}

muda::CBufferView<Float> Codim2DConstitution::BaseInfo::thicknesses() const noexcept
{
    return m_fem->thicknesses.view();
}

muda::CBufferView<Vector3i> Codim2DConstitution::BaseInfo::indices() const noexcept
{
    auto& info = constitution_info();
    return m_fem->codim_2ds.view(info.primitive_offset, info.primitive_count);
}

const FiniteElementMethod::ConstitutionInfo& Codim2DConstitution::BaseInfo::constitution_info() const noexcept
{
    return m_fem->codim_2d_constitution_infos[m_index_in_dim];
}

Float Codim2DConstitution::BaseInfo::dt() const noexcept
{
    return m_dt;
}
}  // namespace uipc::backend::cuda
