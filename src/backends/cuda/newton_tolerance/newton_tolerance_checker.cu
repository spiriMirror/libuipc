#include <newton_tolerance/newton_tolerance_checker.h>

namespace uipc::backend::cuda
{
std::string NewtonToleranceChecker::do_report()
{
    return "";
}
void NewtonToleranceChecker::do_build()
{
    auto& manager = require<NewtonToleranceManager>();

    BuildInfo info;
    do_build(info);  // let derived classes build their specific info

    manager.add_checker(this);  // Register this checker with the manager
}

void NewtonToleranceChecker::init()
{
    InitInfo info;
    do_init(info);  // let derived classes initialize their specific info
}

void NewtonToleranceChecker::pre_newton(PreNewtonInfo& info)
{
    PreNewtonInfo pre_info;
    do_pre_newton(pre_info);  // let derived classes prepare for newton step
}

void NewtonToleranceChecker::check(CheckResultInfo& info)
{
    do_check(info);
}

std::string NewtonToleranceChecker::report()
{
    return do_report();
}
}  // namespace uipc::backend::cuda
