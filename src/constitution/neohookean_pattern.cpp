#include <uipc/constitution/neo_hookean_pattern.h>
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
    uids.push_back(UIDInfo{.uid = 66, .name = "NeoHookeanPattern", .type = string{builtin::FiniteElement}});
    return uids;
}

NeoHookeanPattern::NeoHookeanPattern(const Json& config) noexcept
    : m_config(config)
{
}

void NeoHookeanPattern::apply_to(geometry::SimplicialComplex& sc,
                               const vector<Float>&         X_bars,
                               const ElasticModuli&         moduli,
                               Float                        mass_density,
                               Float                        thickness
                            ) const
{
    Base::apply_to(sc, mass_density, thickness);
    size_t nTri = sc.triangles().size();
    UIPC_ASSERT(sc.dim() == 2, "NeoHookeanPattern only supports 2D simplicial complex");
    UIPC_ASSERT(9 * nTri == X_bars.size(),
                "Rest shape size mismatch");

    const Float mu     = moduli.mu();
    const Float lambda = moduli.lambda();

    auto mu_attr = sc.triangles().find<Float>("mu");
    if (!mu_attr)
        mu_attr = sc.triangles().create<Float>("mu", mu);
    std::ranges::fill(geometry::view(*mu_attr), mu);

    auto lambda_attr = sc.triangles().find<Float>("lambda");
    if (!lambda_attr)
        lambda_attr = sc.triangles().create<Float>("lambda", lambda);
    std::ranges::fill(geometry::view(*lambda_attr), lambda);
    
    auto X_bars_attr = sc.triangles().find<Vector9>("X_rests");
    if (!X_bars_attr)
        X_bars_attr = sc.triangles().create<Vector9>("X_rests");
    auto dst = view(*X_bars_attr);           // span<Vector9>
    UIPC_ASSERT(dst.size() == nTri,
                "attribute size mismatch");

    for (size_t i = 0; i < nTri; ++i)
        for (size_t j = 0; j < 9; ++j)
            dst[i][j] = X_bars[i * 9 + j];
}

Json NeoHookeanPattern::default_config() noexcept
{
    return Json::object();
}

U64 NeoHookeanPattern::get_uid() const noexcept
{
    return 66;
}
}  // namespace uipc::constitution
