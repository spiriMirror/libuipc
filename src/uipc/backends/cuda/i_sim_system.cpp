#include <i_sim_system.h>
#include <typeinfo>
#include <uipc/common/log.h>
namespace uipc::backend::cuda
{
std::string_view ISimSystem::get_name() const noexcept
{
    return typeid(*this).name();
}

void ISimSystem::build()
{
    spdlog::info("Building system: {}", name());
    do_build();
}

std::string_view ISimSystem::name() const noexcept
{
    return get_name();
}
}  // namespace uipc::backend::cuda
