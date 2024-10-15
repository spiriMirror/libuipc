#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
void FEM3DConstitution::init(FiniteElementMethod::FEM3DFilteredInfo& info)
{
    do_init(info);
}

void FEM3DConstitution::do_build(FiniteElementConstitution::BuildInfo& info)
{
    FEM3DConstitution::BuildInfo this_info;
    do_build(this_info);
}

void FEM3DConstitution::do_report_extent(ReportExtentInfo& info)
{
    auto& c_info = constitution_info();
    info.energy_count(c_info.primitive_count);
    info.stencil_dim(dimension() + 1);
}

void FEM3DConstitution::do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info)
{
    FEM3DConstitution::ComputeEnergyInfo this_info{
        &fem(), m_index_in_dim, info.dt(), info.energies()};
    do_compute_energy(this_info);
}

void FEM3DConstitution::do_compute_gradient_hessian(FiniteElementEnergyProducer::ComputeGradientHessianInfo& info)
{
    FEM3DConstitution::ComputeGradientHessianInfo this_info{
        &fem(), m_index_in_dim, info.dt(), info.gradients(), info.hessians()};
    do_compute_gradient_hessian(this_info);
}

const FiniteElementMethod::ConstitutionInfo& FEM3DConstitution::constitution_info() const noexcept
{
    return fem().fem_3d_constitution_infos[m_index_in_dim];
}

IndexT FEM3DConstitution::get_dimension() const noexcept
{
    return 3;
}

Vector2i FEM3DConstitution::get_vertex_offset_count() const noexcept
{
    auto& info = constitution_info();
    return Vector2i{info.vertex_offset, info.vertex_count};
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
}  // namespace uipc::backend::cuda
