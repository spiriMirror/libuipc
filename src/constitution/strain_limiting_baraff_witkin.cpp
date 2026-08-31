#include <uipc/constitution/strain_limiting_baraff_witkin.h>
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
    uids.push_back(UIDInfo{.uid  = 819,
                           .name = "StrainLimitingBaraffWitkinShell",
                           .type = string{builtin::FiniteElement}});
    return uids;
}

StrainLimitingBaraffWitkinShell::StrainLimitingBaraffWitkinShell(const Json& config) noexcept
    : m_config(config)
{
}

void StrainLimitingBaraffWitkinShell::apply_to(geometry::SimplicialComplex& sc,
                                               const ElasticModuli2D& stretch_moduli,
                                               const ElasticModuli2D& shear_moduli,
                                               Float mass_density,
                                               Float thickness,
                                               Float strain_rate) const
{
    Base::apply_to(sc, mass_density, thickness);

    // Cloth stiffness model (same convention as mas-pncg), with independent
    // stretch / shear material parameters:
    //   stretchStiff = E_stretch * (2r) / (1 - nu_stretch^2)
    //                [= (lambda_s + 2*mu_s) * (2r)]
    //   shearStiff   = E_shear / (2*(1 + nu_shear))        [= mu_shear]
    // The stored thickness is the one-sided collision offset. Stretch uses
    // the full shell thickness 2r; shear remains an independently calibrated
    // two-dimensional effective coefficient.
    Float stretch_stiff =
        (stretch_moduli.lambda() + 2 * stretch_moduli.mu()) * (2 * thickness);
    Float shear_stiff = shear_moduli.mu();

    UIPC_ASSERT_THROW(sc.dim() == 2, "StrainLimitingBaraffWitkinShell only supports 2D simplicial complex");

    auto lambda_attr = sc.triangles().find<Float>("lambda");
    if(!lambda_attr)
        lambda_attr = sc.triangles().create<Float>("lambda", stretch_stiff);
    std::ranges::fill(geometry::view(*lambda_attr), stretch_stiff);

    auto mu_attr = sc.triangles().find<Float>("mu");
    if(!mu_attr)
        mu_attr = sc.triangles().create<Float>("mu", shear_stiff);
    std::ranges::fill(geometry::view(*mu_attr), shear_stiff);

    auto strain_rate_attr = sc.triangles().find<Float>("strain_rate");
    if(!strain_rate_attr)
        strain_rate_attr = sc.triangles().create<Float>("strain_rate", strain_rate);
    std::ranges::fill(geometry::view(*strain_rate_attr), strain_rate);
}

Json StrainLimitingBaraffWitkinShell::default_config() noexcept
{
    return Json::object();
}

U64 StrainLimitingBaraffWitkinShell::get_uid() const noexcept
{
    return 819;
}
}  // namespace uipc::constitution
