#pragma once
#include <sim_system.h>
#include <newton_tolerance/newton_tolerance_manager.h>

namespace uipc::backend::cuda
{
class NewtonToleranceChecker : public SimSystem
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

    using CheckResultInfo = NewtonToleranceManager::CheckResultInfo;
    using PreNewtonInfo   = NewtonToleranceManager::PreNewtonInfo;

    class Impl
    {
      public:
    };

  protected:
    virtual void        do_build(BuildInfo& info)          = 0;
    virtual void        do_init(InitInfo& info)            = 0;
    virtual void        do_pre_newton(PreNewtonInfo& info) = 0;
    virtual void        do_check(CheckResultInfo& info)    = 0;
    virtual std::string do_report();

  private:
    friend class NewtonToleranceManager;
    virtual void do_build() override;
    void         init();  // only called by NewtonToleranceManager

    void        pre_newton(PreNewtonInfo& info);
    void        check(CheckResultInfo& info);
    std::string report();
    Impl        m_impl;
};
}  // namespace uipc::backend::cuda