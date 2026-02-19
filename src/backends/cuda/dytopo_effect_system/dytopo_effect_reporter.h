#pragma once
#include <sim_system.h>
#include <dytopo_effect_system/global_dytopo_effect_manager.h>
#include <energy_component_flags.h>

namespace uipc::backend::cuda
{
class DyTopoEffectReporter : public SimSystem
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
    virtual void do_report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info) = 0;
    virtual void do_report_gradient_hessian_extent(
        GlobalDyTopoEffectManager::GradientHessianExtentInfo& info) = 0;
    virtual void do_assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info) = 0;
    virtual void do_compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info) = 0;
    virtual EnergyComponentFlags component_flags() = 0;

  private:
    friend class GlobalDyTopoEffectManager;
    friend class DyTopoEffectLineSearchReporter;
    void init();  // only be called by GlobalDyTopoEffectManager
    void do_build() final override;
    void report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info);
    void report_gradient_hessian_extent(GlobalDyTopoEffectManager::GradientHessianExtentInfo& info);
    void  assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info);
    void  compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info);
    SizeT m_index = ~0ull;
    Impl  m_impl;
};
}  // namespace uipc::backend::cuda
