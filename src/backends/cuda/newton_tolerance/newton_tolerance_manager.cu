#include <newton_tolerance/newton_tolerance_manager.h>
#include <newton_tolerance/newton_tolerance_checker.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(NewtonToleranceManager);

void NewtonToleranceManager::do_build() {}

void NewtonToleranceManager::Impl::init()
{
    auto checkers = tolerance_checkers.view();

    for(auto& checker : checkers)
    {
        checker->init();  // Initialize each registered checker
    }

    report_buffer.reserve(1024);  // Reserve space for report buffer
}

void NewtonToleranceManager::add_checker(NewtonToleranceChecker* checker)
{
    check_state(SimEngineState::BuildSystems, "add_checker()");
    UIPC_ASSERT(checker, "Cannot add null NewtonToleranceChecker");
    m_impl.tolerance_checkers.register_sim_system(*checker);
}

void NewtonToleranceManager::init()
{
    m_impl.init();
}

void NewtonToleranceManager::pre_newton(SizeT frame)
{
    auto checkers = m_impl.tolerance_checkers.view();

    for(auto& checker : checkers)
    {
        PreNewtonInfo pre_info;
        pre_info.m_frame = frame;
        checker->pre_newton(pre_info);
    }
}

void NewtonToleranceManager::check(ResultInfo& info)
{
    auto checkers      = m_impl.tolerance_checkers.view();
    bool all_converged = true;

    m_impl.report_buffer.clear();  // Clear the report buffer for this frame
    for(auto& checker : checkers)
    {
        CheckResultInfo this_info;
        // Input:
        this_info.m_newton_iter = info.m_newton_iter;

        checker->check(this_info);
        std::string report = checker->report();

        fmt::format_to(std::back_inserter(m_impl.report_buffer),
                       "\n    [{}] <{}> {}",
                       this_info.m_converged ? "*" : "x",
                       checker->name(),
                       report);

        // Output:
        if(!this_info.m_converged)
        {
            all_converged = false;
            break;
        }
    }

    logger::info(
        R"(>> Newton Tolerance Checker
============================= [{}] Frame {} Newton Iteration {} ============================{}
--------------------------------------------------------------------------------------------)",
        all_converged ? "*" : "x",
        info.m_frame,
        info.m_newton_iter,
        m_impl.report_buffer);

    info.m_converged = all_converged;
}
}  // namespace uipc::backend::cuda