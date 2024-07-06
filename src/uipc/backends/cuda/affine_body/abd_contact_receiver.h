#pragma once
#include <contact_system/contact_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>

namespace uipc::backend::cuda
{
class ABDContactReceiver final : public ContactReceiver
{
  public:
    using ContactReceiver::ContactReceiver;

    class Impl
    {
      public:
        void receive(GlobalContactManager::ClassifiedContactInfo& info);

        AffineBodyVertexReporter* affine_body_vertex_reporter = nullptr;

        muda::CDoubletVectorView<Float, 3> contact_gradient;
        muda::CTripletMatrixView<Float, 3> contact_hessian;
    };


  protected:
    virtual void do_build(ContactReceiver::BuildInfo& info) override;

  private:
    friend class ABDLinearSubsystem;
    virtual void do_report(GlobalContactManager::ClassifyInfo& info) override;
    virtual void do_receive(GlobalContactManager::ClassifiedContactInfo& info) override;
    Impl m_impl;
};
}  // namespace uipc::backend::cuda
