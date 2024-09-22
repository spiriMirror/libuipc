#include <finite_element/fem_contact_receiver.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMContactReceiver);

void FEMContactReceiver::do_build(ContactReceiver::BuildInfo& info)
{
    m_impl.finite_element_vertex_reporter = &require<FiniteElementVertexReporter>();
}

void FEMContactReceiver::do_report(GlobalContactManager::ClassifyInfo& info)
{
    auto vertex_offset = m_impl.finite_element_vertex_reporter->vertex_offset();
    auto vertex_count  = m_impl.finite_element_vertex_reporter->vertex_count();
    auto v_begin       = vertex_offset;
    auto v_end         = vertex_offset + vertex_count;
    info.range({v_begin, v_end});
}

void FEMContactReceiver::Impl::receive(GlobalContactManager::ClassifiedContactInfo& info)
{
    contact_gradient = info.gradient();
    contact_hessian  = info.hessian();
}

void FEMContactReceiver::do_receive(GlobalContactManager::ClassifiedContactInfo& info)
{
    m_impl.receive(info);
}
}  // namespace uipc::backend::cuda
