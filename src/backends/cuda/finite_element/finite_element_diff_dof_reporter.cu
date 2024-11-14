#include <finite_element/finite_element_diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void FiniteElementDiffDofReporter::do_build(DiffDofReporter::BuildInfo& info)
{
    m_impl.fem = &require<FiniteElementMethod>();
    m_impl.dt  = world().scene().info()["dt"].get<Float>();
    BuildInfo this_info;
    do_build(this_info);
}

void FiniteElementDiffDofReporter::do_assemble(GlobalDiffSimManager::DiffDofInfo& info)
{
    DiffDofInfo this_info{&m_impl, info, m_impl.dt};
    do_assemble(this_info);
}

SizeT FiniteElementDiffDofReporter::DiffDofInfo::frame() const
{
    return m_global_info.frame();
}

IndexT FiniteElementDiffDofReporter::DiffDofInfo::dof_offset(SizeT frame) const
{
    // Frame Dof Offset + FEM Dof Offset => Frame FEM Dof Offset
    return m_global_info.dof_offset(frame) + m_impl->fem->dof_offset(frame);
}

IndexT FiniteElementDiffDofReporter::DiffDofInfo::dof_count(SizeT frame) const
{
    // FEM Dof Count => Frame FEM Dof Count
    return m_impl->fem->dof_count(frame);
}

muda::TripletMatrixView<Float, 1> FiniteElementDiffDofReporter::DiffDofInfo::H() const
{
    return m_global_info.H();
}

Float FiniteElementDiffDofReporter::DiffDofInfo::dt() const
{
    return m_dt;
}
}  // namespace uipc::backend::cuda