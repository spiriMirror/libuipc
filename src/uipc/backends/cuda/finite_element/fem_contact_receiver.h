#pragma once
#include <contact_system/contact_receiver.h>
#include <finite_element/finite_element_vertex_reporter.h>

namespace uipc::backend::cuda
{
class FEMContactReceiver final : public ContactReceiver
{
  public:
    using ContactReceiver::ContactReceiver;

    class Impl
    {
      public:
        void receive(GlobalContactManager::ClassifiedContactInfo& info);

        FiniteElementVertexReporter* finite_element_vertex_reporter = nullptr;

        muda::CDoubletVectorView<Float, 3> contact_gradient;
        muda::CTripletMatrixView<Float, 3> contact_hessian;
    };

    auto contact_graidient() const noexcept { return m_impl.contact_gradient; }
    auto contact_hessian() const noexcept { return m_impl.contact_hessian; }

  protected:
    virtual void do_build(ContactReceiver::BuildInfo& info) override;

  private:
    friend class FEMLinearSubsystem;

    virtual void do_report(GlobalContactManager::ClassifyInfo& info) override;
    virtual void do_receive(GlobalContactManager::ClassifiedContactInfo& info) override;
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
