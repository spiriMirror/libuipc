#include <finite_element/finite_element_diff_parm_reporter.h>
#include <finite_element/finite_element_method.h>

namespace uipc::backend::cuda
{
void FiniteElementDiffParmReporter::do_build(DiffParmReporter::BuildInfo& info)
{
    m_impl.fem     = &require<FiniteElementMethod>();
    m_impl.dt_attr = world().scene().config().find<Float>("dt");
    UIPC_ASSERT(m_impl.dt_attr, "Scene config must have a 'dt' attribute.");
    BuildInfo this_info;
    do_build(this_info);
}

void FiniteElementDiffParmReporter::do_assemble(GlobalDiffSimManager::DiffParmInfo& info)
{
    DiffParmInfo this_info{&m_impl, info, m_impl.dt_attr->view()[0]};
    do_assemble(this_info);
}

SizeT FiniteElementDiffParmReporter::DiffParmInfo::frame() const
{
    return m_global_info.frame();
}

IndexT FiniteElementDiffParmReporter::DiffParmInfo::dof_offset(SizeT frame) const
{
    // Frame Dof Offset + FEM Dof Offset => Frame FEM Dof Offset
    return m_global_info.dof_offset(frame) + m_impl->fem->dof_offset(frame);
}

IndexT FiniteElementDiffParmReporter::DiffParmInfo::dof_count(SizeT frame) const
{
    // FEM Dof Count => Frame FEM Dof Count
    return m_impl->fem->dof_count(frame);
}

muda::TripletMatrixView<Float, 1> FiniteElementDiffParmReporter::DiffParmInfo::pGpP() const
{
    IndexT IF = static_cast<IndexT>(frame());

    auto pGpP = m_global_info.pGpP();

    auto row_offset = dof_offset(IF);
    auto col_offset = 0;

    auto row_count = dof_count(IF);
    auto col_count = pGpP.extent().y;

    return pGpP.submatrix({row_offset, col_offset}, {row_count, col_count});
}

Float FiniteElementDiffParmReporter::DiffParmInfo::dt() const
{
    return m_impl->dt_attr->view()[0];
}
}  // namespace uipc::backend::cuda
