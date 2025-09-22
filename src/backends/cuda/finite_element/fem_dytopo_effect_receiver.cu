#include <finite_element/fem_dytopo_effect_receiver.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMDyTopoEffectReceiver);

void FEMDyTopoEffectReceiver::do_build(DyTopoEffectReceiver::BuildInfo& info)
{
    m_impl.finite_element_vertex_reporter = &require<FiniteElementVertexReporter>();
}

void FEMDyTopoEffectReceiver::do_report(GlobalDyTopoEffectManager::ClassifyInfo& info)
{
    auto vertex_offset = m_impl.finite_element_vertex_reporter->vertex_offset();
    auto vertex_count  = m_impl.finite_element_vertex_reporter->vertex_count();
    auto v_begin       = vertex_offset;
    auto v_end         = vertex_offset + vertex_count;
    info.range({v_begin, v_end});
}

void FEMDyTopoEffectReceiver::Impl::receive(GlobalDyTopoEffectManager::ClassifiedDyTopoEffectInfo& info)
{
    gradients = info.gradients();
    hessians  = info.hessians();
}

void FEMDyTopoEffectReceiver::do_receive(GlobalDyTopoEffectManager::ClassifiedDyTopoEffectInfo& info)
{
    m_impl.receive(info);
}
}  // namespace uipc::backend::cuda
