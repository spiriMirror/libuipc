#include <affine_body/abd_dytopo_effect_receiver.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDDyTopoEffectReceiver);

void ABDDyTopoEffectReceiver::do_build(DyTopoEffectReceiver::BuildInfo& info)
{
    m_impl.affine_body_vertex_reporter = &require<AffineBodyVertexReporter>();
}

void ABDDyTopoEffectReceiver::do_report(GlobalDyTopoEffectManager::ClassifyInfo& info)
{
    auto vertex_offset = m_impl.affine_body_vertex_reporter->vertex_offset();
    auto vertex_count  = m_impl.affine_body_vertex_reporter->vertex_count();
    auto v_begin       = vertex_offset;
    auto v_end         = v_begin + vertex_count;
    info.range({v_begin, v_end});
}

void ABDDyTopoEffectReceiver::Impl::receive(GlobalDyTopoEffectManager::ClassifiedDyTopoEffectInfo& info)
{
    gradients = info.gradients();
    hessians  = info.hessians();
}

void ABDDyTopoEffectReceiver::do_receive(GlobalDyTopoEffectManager::ClassifiedDyTopoEffectInfo& info)
{
    m_impl.receive(info);
}
}  // namespace uipc::backend::cuda
