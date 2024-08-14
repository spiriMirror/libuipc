#include <affine_body/abd_contact_receiver.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDContactReceiver);

void ABDContactReceiver::do_build(ContactReceiver::BuildInfo& info)
{
    m_impl.affine_body_vertex_reporter = &require<AffineBodyVertexReporter>();
}

void ABDContactReceiver::do_report(GlobalContactManager::ClassifyInfo& info)
{
    auto vertex_offset = m_impl.affine_body_vertex_reporter->vertex_offset();
    auto vertex_count  = m_impl.affine_body_vertex_reporter->vertex_count();
    auto v_begin       = vertex_offset;
    auto v_end         = v_begin + vertex_count;
    info.range({v_begin, v_end});
}

void ABDContactReceiver::Impl::receive(GlobalContactManager::ClassifiedContactInfo& info)
{
    contact_gradient = info.gradient();
    contact_hessian  = info.hessian();
}

void ABDContactReceiver::do_receive(GlobalContactManager::ClassifiedContactInfo& info)
{
    m_impl.receive(info);
}
}  // namespace uipc::backend::cuda
