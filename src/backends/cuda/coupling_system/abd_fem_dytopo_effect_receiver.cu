#include <coupling_system/abd_fem_dytopo_effect_receiver.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDFEMDyTopoEffectReceiver);

void ABDFEMDyTopoEffectReceiver::do_build(BuildInfo& info)
{
    m_impl.affine_body_vertex_reporter = &require<AffineBodyVertexReporter>();
    m_impl.finite_element_vertex_reporter = &require<FiniteElementVertexReporter>();
}

void ABDFEMDyTopoEffectReceiver::do_report(GlobalDyTopoEffectManager::ClassifyInfo& info)
{
    IndexT fem_v_offset = m_impl.finite_element_vertex_reporter->vertex_offset();
    IndexT fem_v_count = m_impl.finite_element_vertex_reporter->vertex_count();
    auto   fem_v_begin = fem_v_offset;
    auto   fem_v_end   = fem_v_offset + fem_v_count;


    IndexT abd_v_offset = m_impl.affine_body_vertex_reporter->vertex_offset();
    IndexT abd_v_count  = m_impl.affine_body_vertex_reporter->vertex_count();
    auto   abd_v_begin  = abd_v_offset;
    auto   abd_v_end    = abd_v_offset + abd_v_count;


    info.range({abd_v_begin, abd_v_end}, {fem_v_begin, fem_v_end});
}

void ABDFEMDyTopoEffectReceiver::do_receive(GlobalDyTopoEffectManager::ClassifiedDyTopoEffectInfo& info)
{
    m_impl.hessians = info.hessians();
}
}  // namespace uipc::backend::cuda
