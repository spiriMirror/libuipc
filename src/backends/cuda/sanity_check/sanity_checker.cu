#include <sanity_check/sanity_checker.h>
#include <sanity_check/sanity_check_manager.h>

namespace uipc::backend::cuda
{
void SanityChecker::do_build()
{
    auto& manager = require<SanityCheckManager>();

    BuildInfo info;
    do_build(info);

    manager.add_checker(this);
}

void SanityChecker::init()
{
    InitInfo info;
    do_init(info);
}

void SanityChecker::check(CheckInfo& info)
{
    do_check(info);
}
}  // namespace uipc::backend::cuda
