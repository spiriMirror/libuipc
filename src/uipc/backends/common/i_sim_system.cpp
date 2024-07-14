#include <typeinfo>
#include <uipc/backends/common/i_sim_system.h>

namespace uipc::backend
{
void ISimSystem::build()
{
    //spdlog::info("Building system: {}", name());
    do_build();
}

void ISimSystem::make_engine_aware()
{
    set_engine_aware();
}

void ISimSystem::invalidate() noexcept
{
    set_invalid();
}

bool ISimSystem::is_valid() const noexcept
{
    return get_valid();
}

bool ISimSystem::is_building() const noexcept
{
    return get_is_building();
}

span<ISimSystem* const> ISimSystem::dependencies() const noexcept
{
    return get_dependencies();
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
}  // namespace uipc::backend
