#include <uipc/constitutions/stable_neo_hookean.h>

namespace uipc::constitution
{
StableNeoHookean::StableNeoHookean(const Json& config) noexcept {}

void StableNeoHookean::apply_to(geometry::SimplicialComplex& sc) const 
{
    Base::apply_to(sc);
}

Json StableNeoHookean::default_config() noexcept
{
    return Json::object();
}

U64 StableNeoHookean::get_uid() const noexcept
{
    return 9;
}

std::string_view StableNeoHookean::get_name() const noexcept
{
    return builtin::ConstitutionUIDRegister::instance().find(get_uid()).name;
}

world::ConstitutionType StableNeoHookean::get_type() const noexcept
{
    return world::ConstitutionType::FiniteElement;
}
}  // namespace uipc::constitution
