#include <sanity_check/sanity_check_manager.h>
#include <sanity_check/sanity_checker.h>
#include <sim_engine.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/timer.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(SanityCheckManager);

void SanityCheckManager::do_build()
{
    // Nothing to configure at build time;
    // the World decides whether to call this collection based on sanity_check/method.
}

void SanityCheckManager::Impl::init()
{
    auto checker_view = checkers.view();
    for(auto&& [i, checker] : enumerate(checker_view))
        checker->init();
}

void SanityCheckManager::init()
{
    m_impl.init();
}

void SanityCheckManager::check(SizeT frame, SizeT newton_iter)
{
    Timer timer{"GPU SanityCheck"};

    CheckInfo info;
    info.m_frame       = frame;
    info.m_newton_iter = newton_iter;

    auto checker_view = m_impl.checkers.view();
    for(auto&& [i, checker] : enumerate(checker_view))
    {
        Timer checker_timer{fmt::format("SanityCheck::{}", checker->name())};
        checker->check(info);
    }
}

// ISanityCheckerCollection interface
void SanityCheckManager::build(core::internal::Scene& /*s*/)
{
    // GPU checkers are already built as SimSystems during engine init.
    // Nothing to do here.
}

core::SanityCheckResult SanityCheckManager::check(core::SanityCheckMessageCollection& /*msg*/) const
{
    // Delegate to the internal check mechanism.
    // GPU checkers currently log violations directly rather than producing SanityCheckMessages.
    // We use frame=0, newton_iter=0 for init-time sanity check.
    const_cast<SanityCheckManager*>(this)->check(0, 0);
    return core::SanityCheckResult::Success;
}

void SanityCheckManager::add_checker(SanityChecker* checker)
{
    check_state(SimEngineState::BuildSystems, "add_checker()");
    UIPC_ASSERT(checker != nullptr, "Cannot add null SanityChecker");
    m_impl.checkers.register_sim_system(*checker);
}
}  // namespace uipc::backend::cuda
