#pragma once
#include <contact_system/contact_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <finite_element/finite_element_vertex_reporter.h>
namespace uipc::backend::cuda
{
class FEMABDContactReceiver final : public ContactReceiver
{
  public:
    using ContactReceiver::ContactReceiver;
    class Impl
    {
      public:
        AffineBodyVertexReporter*    affine_body_vertex_reporter    = nullptr;
        FiniteElementVertexReporter* finite_element_vertex_reporter = nullptr;
        muda::CTripletMatrixView<Float, 3> contact_hessian;
    };

    auto contact_hessian() const noexcept { return m_impl.contact_hessian; }

  private:
    virtual void do_report(GlobalContactManager::ClassifyInfo& info) override;
    virtual void do_receive(GlobalContactManager::ClassifiedContactInfo& info) override;

    Impl m_impl;

    // Inherited via ContactReceiver
    void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda
