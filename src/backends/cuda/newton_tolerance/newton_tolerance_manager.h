#pragma once
#include <sim_system.h>

namespace uipc::backend::cuda
{
class NewtonToleranceChecker;

class NewtonToleranceManager final : public SimSystem
{
  public:
    using SimSystem::SimSystem;

    class ResultInfo
    {
      public:
        bool converged() const { return m_converged; }
        void frame(SizeT frame) { m_frame = frame; }
        void newton_iter(IndexT count) { m_newton_iter = count; }

      private:
        friend class NewtonToleranceManager;
        bool   m_converged   = true;  // true if all checkers converged
        IndexT m_newton_iter = 0;
        SizeT  m_frame       = 0;  // current frame
    };

    class CheckResultInfo
    {
      public:
        // Finished iteration count
        IndexT newton_iter() const { return m_newton_iter; }
        void   converged(bool converged) { m_converged = converged; }

      private:
        friend class NewtonToleranceManager;
        bool   m_converged   = true;  // true if this checker converged
        IndexT m_newton_iter = 0;
    };

    class PreNewtonInfo
    {
      public:
        SizeT frame() const { return m_frame; }

      private:
        friend class NewtonToleranceManager;
        SizeT m_frame = 0;  // current frame
    };

    class Impl
    {
      public:
        void init();

        std::string                                     report_buffer;
        SimSystemSlotCollection<NewtonToleranceChecker> tolerance_checkers;
    };

    virtual void do_build() override;

  private:
    friend class SimEngine;
    void init();  // only be called by SimEngine
    void pre_newton(SizeT frame);
    void check(ResultInfo& info);

    friend class NewtonToleranceChecker;
    void add_checker(NewtonToleranceChecker* checker);
    Impl m_impl;
};
}  // namespace uipc::backend::cuda