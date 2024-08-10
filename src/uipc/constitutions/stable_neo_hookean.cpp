#include <uipc/constitutions/stable_neo_hookean.h>
#include <uipc/geometry/utils/compute_vertex_mass.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/constitutions/conversion.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
StableNeoHookean::StableNeoHookean(const Json& config) noexcept {}

void StableNeoHookean::apply_to(geometry::SimplicialComplex& sc,
                                const StableNeoHookeanParms& parms,
                                Float                        mass_density) const
{
    Base::apply_to(sc);

    auto mu     = parms.m_mu;
    auto lambda = parms.m_lambda;

    UIPC_ASSERT(sc.dim() == 3, "Now StableNeoHookean only supports 3D simplicial complex");

    auto mu_attr = sc.tetrahedra().find<Float>("mu");
    if(!mu_attr)
        mu_attr = sc.tetrahedra().create<Float>("mu", mu);
    std::ranges::fill(geometry::view(*mu_attr), mu);

    auto lambda_attr = sc.tetrahedra().find<Float>("lambda");
    if(!lambda_attr)
        lambda_attr = sc.tetrahedra().create<Float>("lambda", lambda);
    std::ranges::fill(geometry::view(*lambda_attr), lambda);

    auto is_fixed = sc.vertices().find<IndexT>(builtin::is_fixed);
    if(!is_fixed)
        is_fixed = sc.vertices().create<IndexT>(builtin::is_fixed, 0);

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

StableNeoHookeanParms StableNeoHookeanParms::EG(Float E, Float G)
{
    StableNeoHookeanParms parms;
    EG_to_lame(E, G, parms.m_lambda, parms.m_mu, parms.m_poisson_ratio);
    return parms;
}

StableNeoHookeanParms StableNeoHookeanParms::ML(Float mu, Float lambda)
{
    StableNeoHookeanParms parms;
    parms.m_mu     = mu;
    parms.m_lambda = lambda;
    lame_to_poisson(parms.m_lambda, parms.m_mu, parms.m_poisson_ratio);
    return parms;
}
StableNeoHookeanParms StableNeoHookeanParms::EP(Float E, Float poisson)
{
    StableNeoHookeanParms parms;
    if(poisson == 0.5)
    {
        throw Exception("Poission Rate can't be 0.5");
    }
    parms.m_poisson_ratio = poisson;
    EP_to_lame(E, poisson, parms.m_lambda, parms.m_mu);
    return parms;
}
}  // namespace uipc::constitution
