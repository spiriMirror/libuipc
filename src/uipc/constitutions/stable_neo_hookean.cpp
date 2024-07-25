#include <uipc/constitutions/stable_neo_hookean.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>
namespace uipc::constitution
{
StableNeoHookean::StableNeoHookean(const Json& config) noexcept {}

void StableNeoHookean::apply_to(geometry::SimplicialComplex& sc, Float mass_density) const
{
    Base::apply_to(sc);
    geometry::compute_vertex_mass(sc, mass_density);
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
