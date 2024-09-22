#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
void FEM3DConstitution::retrieve(FiniteElementMethod::FEM3DFilteredInfo& info)
{
    do_retrieve(info);
}

void FEM3DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    FEM3DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void FEM3DConstitution::do_compute_energy(FiniteElementMethod::ComputeEnergyInfo& info)
{
    FEM3DConstitution::ComputeEnergyInfo this_info{&fem(), m_index_in_dim, info.dt()};
    do_compute_energy(this_info);
}

void FEM3DConstitution::do_compute_gradient_hessian(FiniteElementMethod::ComputeGradientHessianInfo& info)
{
    FEM3DConstitution::ComputeGradientHessianInfo this_info{&fem(), m_index_in_dim, info.dt()};
    do_compute_gradient_hessian(this_info);
}

IndexT FEM3DConstitution::get_dimension() const
{
    return 3;
}

muda::CBufferView<Vector4i> FEM3DConstitution::BaseInfo::indices() const noexcept
{
    auto& info = constitution_info();
    return m_fem->tets.view(info.primitive_offset, info.primitive_count);
}

muda::CBufferView<Vector3> FEM3DConstitution::BaseInfo::xs() const noexcept
{
    return m_fem->xs.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Vector3> FEM3DConstitution::BaseInfo::x_bars() const noexcept
{
    return m_fem->x_bars.view();  // must return full buffer, because the indices index into the full buffer
}

muda::CBufferView<Matrix3x3> FEM3DConstitution::BaseInfo::Dm_invs() const noexcept
{
    auto& info = constitution_info();
    return m_fem->Dm3x3_invs.view(info.primitive_offset, info.primitive_count);
}

muda::CBufferView<Float> FEM3DConstitution::BaseInfo::rest_volumes() const noexcept
{
    auto& info = constitution_info();
    return m_fem->rest_volumes.view(info.primitive_offset, info.primitive_count);
}

const FiniteElementMethod::ConstitutionInfo& FEM3DConstitution::BaseInfo::constitution_info() const noexcept
{
    return m_fem->fem_3d_constitution_infos[m_index_in_dim];
}

muda::BufferView<Vector12> FEM3DConstitution::ComputeGradientHessianInfo::gradient() const noexcept
{
    auto& info = constitution_info();
    return m_fem->G12s.view(info.primitive_offset, info.primitive_count);
}

muda::BufferView<Matrix12x12> FEM3DConstitution::ComputeGradientHessianInfo::hessian() const noexcept
{
    auto& info = constitution_info();
    return m_fem->H12x12s.view(info.primitive_offset, info.primitive_count);
}

muda::BufferView<Float> FEM3DConstitution::ComputeEnergyInfo::element_energies() const noexcept
{
    auto& info = constitution_info();
    return m_fem->fem_3d_elastic_energies.view(info.primitive_offset, info.primitive_count);
}
}  // namespace uipc::backend::cuda
