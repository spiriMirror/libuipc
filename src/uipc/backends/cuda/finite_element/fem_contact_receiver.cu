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
    info.gradient_range({vertex_offset, vertex_count});
    info.hessian_range({vertex_offset, vertex_count}, {vertex_offset, vertex_count});
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
