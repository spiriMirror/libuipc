#include <uipc/constitution/plastic_discrete_shell_bending.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/common/log.h>
#include <cmath>

namespace uipc::constitution
{
constexpr U64 PlasticDiscreteShellBendingUID = 31;

REGISTER_CONSTITUTION_UIDS()
{
    list<builtin::UIDInfo> uid_infos;
    builtin::UIDInfo       info;
    info.uid  = PlasticDiscreteShellBendingUID;
    info.name = "PlasticDiscreteShellBending";
    info.type = string{builtin::FiniteElement};
    uid_infos.push_back(info);
    return uid_infos;
}

PlasticDiscreteShellBending::PlasticDiscreteShellBending(const Json& json)
    : m_config{json}
{
}

void PlasticDiscreteShellBending::apply_to(geometry::SimplicialComplex& sc,
                                           Float                        bending_stiffness_v,
                                           Float                        yield_threshold_v,
                                           Float                        hardening_modulus_v)
{
    UIPC_ASSERT(std::isfinite(bending_stiffness_v),
                "PlasticDiscreteShellBending requires a finite bending_stiffness, got {}",
                bending_stiffness_v);
    UIPC_ASSERT(std::isfinite(yield_threshold_v) && yield_threshold_v >= 0.0,
                "PlasticDiscreteShellBending requires a finite yield_threshold >= 0, got {}",
                yield_threshold_v);
    UIPC_ASSERT(std::isfinite(hardening_modulus_v) && hardening_modulus_v >= 0.0,
                "PlasticDiscreteShellBending requires hardening_modulus to be finite and >= 0, got {}",
                hardening_modulus_v);

    Base::apply_to(sc);

    auto bs = sc.edges().find<Float>("bending_stiffness");
    if(!bs)
        bs = sc.edges().create<Float>("bending_stiffness");
    std::ranges::fill(geometry::view(*bs), bending_stiffness_v);

    auto yield_threshold = sc.edges().find<Float>("bending_yield_threshold");
    if(!yield_threshold)
        yield_threshold = sc.edges().create<Float>("bending_yield_threshold");
    std::ranges::fill(geometry::view(*yield_threshold), yield_threshold_v);

    auto hardening_modulus = sc.edges().find<Float>("bending_hardening_modulus");
    if(!hardening_modulus)
        hardening_modulus = sc.edges().create<Float>("bending_hardening_modulus");
    std::ranges::fill(geometry::view(*hardening_modulus), hardening_modulus_v);
}

U64 PlasticDiscreteShellBending::get_uid() const noexcept
{
    return PlasticDiscreteShellBendingUID;
}

Json PlasticDiscreteShellBending::default_config()
{
    return Json::object();
}
}  // namespace uipc::constitution
