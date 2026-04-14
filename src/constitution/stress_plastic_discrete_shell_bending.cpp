#include <uipc/constitution/stress_plastic_discrete_shell_bending.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/common/log.h>
#include <cmath>

namespace uipc::constitution
{
constexpr U64 StressPlasticDiscreteShellBendingUID = 32;

REGISTER_CONSTITUTION_UIDS()
{
    list<builtin::UIDInfo> uid_infos;
    builtin::UIDInfo       info;
    info.uid  = StressPlasticDiscreteShellBendingUID;
    info.name = "StressPlasticDiscreteShellBending";
    info.type = string{builtin::FiniteElement};
    uid_infos.push_back(info);
    return uid_infos;
}

StressPlasticDiscreteShellBending::StressPlasticDiscreteShellBending(const Json& json)
    : m_config{json}
{
}

void StressPlasticDiscreteShellBending::apply_to(geometry::SimplicialComplex& sc,
                                                 Float                        bending_stiffness_v,
                                                 Float                        yield_stress_v,
                                                 Float hardening_modulus_v)
{
    UIPC_ASSERT_THROW(std::isfinite(bending_stiffness_v),
                "StressPlasticDiscreteShellBending requires a finite bending_stiffness, got {}",
                bending_stiffness_v);
    UIPC_ASSERT_THROW(std::isfinite(yield_stress_v) && yield_stress_v >= 0.0,
                "StressPlasticDiscreteShellBending requires a finite yield_stress >= 0, got {}",
                yield_stress_v);
    UIPC_ASSERT_THROW(std::isfinite(hardening_modulus_v) && hardening_modulus_v >= 0.0,
                "StressPlasticDiscreteShellBending requires hardening_modulus to be finite and >= 0, got {}",
                hardening_modulus_v);

    Base::apply_to(sc);

    auto bs = sc.edges().find<Float>("bending_stiffness");
    if(!bs)
        bs = sc.edges().create<Float>("bending_stiffness");
    std::ranges::fill(geometry::view(*bs), bending_stiffness_v);

    auto yield_stress = sc.edges().find<Float>("bending_yield_stress");
    if(!yield_stress)
        yield_stress = sc.edges().create<Float>("bending_yield_stress");
    std::ranges::fill(geometry::view(*yield_stress), yield_stress_v);

    auto hardening_modulus = sc.edges().find<Float>("bending_hardening_modulus");
    if(!hardening_modulus)
        hardening_modulus = sc.edges().create<Float>("bending_hardening_modulus");
    std::ranges::fill(geometry::view(*hardening_modulus), hardening_modulus_v);
}

U64 StressPlasticDiscreteShellBending::get_uid() const noexcept
{
    return StressPlasticDiscreteShellBendingUID;
}

Json StressPlasticDiscreteShellBending::default_config()
{
    return Json::object();
}
}  // namespace uipc::constitution
