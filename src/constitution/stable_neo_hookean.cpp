#include <uipc/constitution/stable_neo_hookean.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/constitution/conversion.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
REGISTER_CONSTITUTION_UIDS()
{
    using namespace uipc::builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = 10,
                           .name = "StableNeoHookean",
                           .type = string{builtin::FiniteElement}});
    return uids;
}

StableNeoHookean::StableNeoHookean(const Json& config) noexcept
    : m_config(config)
{
}

void StableNeoHookean::apply_to(geometry::SimplicialComplex& sc,
                                const ElasticModuli&         moduli,
                                Float                        mass_density) const
{
    Base::apply_to(sc, mass_density);

    auto mu     = moduli.mu();
    auto lambda = moduli.lambda();

    // SNK1 linearizes to 2*mu*epsilon + (lambda-mu)*trace(epsilon)*I.
    // Match the supplied physical Lame parameters, without the old log-barrier
    // model's 4/3 and 5/6 reparameterization.
    auto snh_mu     = mu;
    auto snh_lambda = lambda + mu;

    UIPC_ASSERT_THROW(sc.dim() == 3, "StableNeoHookean only supports 3D simplicial complex");

    auto mu_attr = sc.tetrahedra().find<Float>("mu");
    if(!mu_attr)
        mu_attr = sc.tetrahedra().create<Float>("mu", snh_mu);
    std::ranges::fill(geometry::view(*mu_attr), snh_mu);

    auto lambda_attr = sc.tetrahedra().find<Float>("lambda");
    if(!lambda_attr)
        lambda_attr = sc.tetrahedra().create<Float>("lambda", snh_lambda);
    std::ranges::fill(geometry::view(*lambda_attr), snh_lambda);
}

Json StableNeoHookean::default_config() noexcept
{
    return Json::object();
}

U64 StableNeoHookean::get_uid() const noexcept
{
    return 10;
}
}  // namespace uipc::constitution
