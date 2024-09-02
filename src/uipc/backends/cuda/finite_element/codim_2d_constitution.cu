#include <finite_element/codim_2d_constitution.h>

namespace uipc::backend::cuda
{
void Codim2DConstitution::retrieve(FiniteElementMethod::Codim2DFilteredInfo& info)
{
    do_retrieve(info);
}

void Codim2DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    Codim2DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void Codim2DConstitution::do_compute_energy(FiniteElementMethod::ComputeEnergyInfo& info)
{
    Codim2DConstitution::ComputeEnergyInfo this_info{&fem(), m_index_in_dim, info.dt()};
    do_compute_energy(this_info);
}

void Codim2DConstitution::do_compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info)
{
    Codim2DConstitution::ComputeGradientHessianInfo this_info{
        &fem(), m_index_in_dim, info.dt()};
    do_compute_gradient_hessian(this_info);
}

IndexT Codim2DConstitution::get_dimension() const
{
    return 2;
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

muda::BufferView<Float> Codim2DConstitution::ComputeEnergyInfo::element_energies() const noexcept
{
    auto& info = constitution_info();
    return m_fem->codim_2d_elastic_energies.view(info.primitive_offset, info.primitive_count);
}

muda::BufferView<Vector9> Codim2DConstitution::ComputeGradientHessianInfo::gradient() const noexcept
{
    auto& info = constitution_info();
    return m_fem->G9s.view(info.primitive_offset, info.primitive_count);
}

muda::BufferView<Matrix9x9> Codim2DConstitution::ComputeGradientHessianInfo::hessian() const noexcept
{
    auto& info = constitution_info();
    return m_fem->H9x9s.view(info.primitive_offset, info.primitive_count);
}
}  // namespace uipc::backend::cuda
