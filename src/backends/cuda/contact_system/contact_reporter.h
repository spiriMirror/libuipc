#pragma once
#include <sim_system.h>
#include <contact_system/global_contact_manager.h>
namespace uipc::backend::cuda
{
class ContactReporter : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class BuildInfo
    {
      public:
    };

    class InitInfo
    {
      public:
    };

    class Impl
    {
      public:
        muda::CBufferView<Float>           energies;
        muda::CDoubletVectorView<Float, 3> gradients;
        muda::CTripletMatrixView<Float, 3> hessians;
    };

  protected:
    virtual void do_build(BuildInfo& info) = 0;
    virtual void do_init(InitInfo&);
    virtual void do_report_energy_extent(GlobalContactManager::EnergyExtentInfo& info) = 0;
    virtual void do_report_gradient_hessian_extent(
        GlobalContactManager::GradientHessianExtentInfo& info) = 0;
    virtual void do_assemble(GlobalContactManager::GradientHessianInfo& info) = 0;
    virtual void do_compute_energy(GlobalContactManager::EnergyInfo& info) = 0;

  private:
    friend class GlobalContactManager;
    friend class ContactLineSearchReporter;
    void init();  // only be called by GlobalContactManager
    void do_build() final override;
    void report_energy_extent(GlobalContactManager::EnergyExtentInfo& info);
    void report_gradient_hessian_extent(GlobalContactManager::GradientHessianExtentInfo& info);
    void  assemble(GlobalContactManager::GradientHessianInfo& info);
    void  compute_energy(GlobalContactManager::EnergyInfo& info);
    SizeT m_index = ~0ull;
    Impl  m_impl;
};
}  // namespace uipc::backend::cuda
