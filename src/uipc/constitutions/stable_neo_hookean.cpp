#include <uipc/constitutions/stable_neo_hookean.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>
#include <uipc/builtin/attribute_name.h>
namespace uipc::constitution
{
StableNeoHookean::StableNeoHookean(const Json& config) noexcept {}

void StableNeoHookean::apply_to(geometry::SimplicialComplex& sc, Float mu, Float lambda, Float mass_density) const
{
    Base::apply_to(sc);

    auto mu_attr = sc.vertices().find<Float>("mu");
    if(!mu_attr)
        mu_attr = sc.vertices().create<Float>("mu", mu);
    std::ranges::fill(geometry::view(*mu_attr), mu);

    auto lambda_attr = sc.vertices().find<Float>("lambda");
    if(!lambda_attr)
        lambda_attr = sc.vertices().create<Float>("lambda", lambda);
    std::ranges::fill(geometry::view(*lambda_attr), lambda);

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
