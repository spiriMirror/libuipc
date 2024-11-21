#include <uipc/constitution/neo_hookean_shell.h>
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
    uids.push_back(UIDInfo{.uid = 11, .name = "NeoHookeanShell", .type = string{builtin::FiniteElement}});
    return uids;
}

NeoHookeanShell::NeoHookeanShell(const Json& config) noexcept
    : m_config(config)
{
}

void NeoHookeanShell::apply_to(geometry::SimplicialComplex& sc,
                               const ElasticModuli&         moduli,
                               Float                        mass_density,
                               Float                        thickness) const
{
    Base::apply_to(sc, mass_density, thickness);

    auto mu     = moduli.mu();
    auto lambda = moduli.lambda();

    UIPC_ASSERT(sc.dim() == 2, "NeoHookeanShell only supports 2D simplicial complex");

    auto mu_attr = sc.triangles().find<Float>("mu");
    if(!mu_attr)
        mu_attr = sc.triangles().create<Float>("mu", mu);
    std::ranges::fill(geometry::view(*mu_attr), mu);

    auto lambda_attr = sc.triangles().find<Float>("lambda");
    if(!lambda_attr)
        lambda_attr = sc.triangles().create<Float>("lambda", lambda);
    std::ranges::fill(geometry::view(*lambda_attr), lambda);
}

Json NeoHookeanShell::default_config() noexcept
{
    return Json::object();
}

U64 NeoHookeanShell::get_uid() const noexcept
{
    return 11;
}
}  // namespace uipc::constitution
