#pragma once
#include <sim_system.h>
#include <uipc/core/i_sanity_checker.h>

namespace uipc::backend::cuda
{
class SanityChecker;

class SanityCheckManager final : public SimSystem
                                , public core::ISanityCheckerCollection
{
  public:
    using SimSystem::SimSystem;

    class CheckInfo
    {
      public:
        SizeT frame() const noexcept { return m_frame; }
        SizeT newton_iter() const noexcept { return m_newton_iter; }

      private:
        friend class SanityCheckManager;
        SizeT m_frame       = 0;
        SizeT m_newton_iter = 0;
    };

    class Impl
    {
      public:
        void init();

        SimSystemSlotCollection<SanityChecker> checkers;
    };

    virtual void do_build() override;

    // ISanityCheckerCollection interface
    virtual void              build(core::internal::Scene& s) override;
    virtual core::SanityCheckResult check(core::SanityCheckMessageCollection& msg) const override;

  private:
    friend class SimEngine;
    void init();
    void check(SizeT frame, SizeT newton_iter);

    friend class SanityChecker;
    void add_checker(SanityChecker* checker);

    Impl m_impl;
};
}  // namespace uipc::backend::cuda
