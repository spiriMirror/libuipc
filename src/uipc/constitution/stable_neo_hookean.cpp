#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
StableNeoHookean::StableNeoHookean(const Json& config) noexcept {}

void StableNeoHookean::apply_to(geometry::SimplicialComplex& sc,
                                const ElasticModuli&         moduli,
                                Float                        mass_density) const
{
    Base::apply_to(sc, mass_density);

    auto mu     = moduli.mu();
    auto lambda = moduli.lambda();

    UIPC_ASSERT(sc.dim() == 3, "Now StableNeoHookean only supports 3D simplicial complex");

    auto mu_attr = sc.tetrahedra().find<Float>("mu");
    if(!mu_attr)
        mu_attr = sc.tetrahedra().create<Float>("mu", mu);
    std::ranges::fill(geometry::view(*mu_attr), mu);

    auto lambda_attr = sc.tetrahedra().find<Float>("lambda");
    if(!lambda_attr)
        lambda_attr = sc.tetrahedra().create<Float>("lambda", lambda);
    std::ranges::fill(geometry::view(*lambda_attr), lambda);
}

Json StableNeoHookean::default_config() noexcept
{
    return Json::object();
}

U64 StableNeoHookean::get_uid() const noexcept
{
    return 10;
}

std::string_view StableNeoHookean::get_name() const noexcept
{
    return builtin::ConstitutionUIDRegister::instance().find(get_uid()).name;
}
}  // namespace uipc::constitution
