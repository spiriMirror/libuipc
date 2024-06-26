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
    //spdlog::info("Building system: {}", name());
    do_build();
}

void ISimSystem::make_engine_aware()
{
    set_engine_aware();
}

std::string_view ISimSystem::name() const noexcept
{
    return get_name();
}

bool ISimSystem::is_engine_aware() const noexcept
{
    return get_engine_aware();
}

Json ISimSystem::to_json() const
{
    return do_to_json();
}
}  // namespace uipc::backend::cuda
