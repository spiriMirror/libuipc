#pragma once
#include <dytopo_effect_system/dytopo_effect_receiver.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <finite_element/finite_element_vertex_reporter.h>
namespace uipc::backend::cuda
{
class ABDFEMDyTopoEffectReceiver final : public DyTopoEffectReceiver
{
  public:
    using DyTopoEffectReceiver::DyTopoEffectReceiver;
    class Impl
    {
      public:
        AffineBodyVertexReporter*    affine_body_vertex_reporter    = nullptr;
        FiniteElementVertexReporter* finite_element_vertex_reporter = nullptr;

        muda::CTripletMatrixView<Float, 3> hessians;
    };

    auto hessians() const noexcept { return m_impl.hessians; }

  private:
    virtual void do_report(GlobalDyTopoEffectManager::ClassifyInfo& info) override;
    virtual void do_receive(GlobalDyTopoEffectManager::ClassifiedDyTopoEffectInfo& info) override;

    Impl m_impl;

    // Inherited via DyTopoEffectReceiver
    void do_build(BuildInfo& info) override;
};
}  // namespace uipc::backend::cuda
