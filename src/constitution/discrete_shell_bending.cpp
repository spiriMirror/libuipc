#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/log.h>

namespace uipc::constitution
{
constexpr U64 DiscreteShellBendingUID = 17;

REGISTER_CONSTITUTION_UIDS()
{
    list<builtin::UIDInfo> uid_infos;
    builtin::UIDInfo       info;
    info.uid  = DiscreteShellBendingUID;
    info.name = "DiscreteShellBending";
    info.type = string{builtin::FiniteElement};
    uid_infos.push_back(info);
    return uid_infos;
}

DiscreteShellBending::DiscreteShellBending(const Json& json)
    : m_config{json}
{
}

void DiscreteShellBending::apply_to(geometry::SimplicialComplex& sc, Float bending_stiffness_v)
{
    Base::apply_to(sc);
    auto bs = sc.edges().find<Float>("bending_stiffness");
    if(!bs)
    {
        bs = sc.edges().create<Float>("bending_stiffness");
    }
    auto bs_view = geometry::view(*bs);
    std::ranges::fill(bs_view, bending_stiffness_v);
}

Float DiscreteShellBending::bending_stiffness(Float young_modulus,
                                              Float poisson_ratio,
                                              Float thickness) noexcept
{
    // `thickness` is the one-sided collision offset r. Classical shell
    // bending uses the full material thickness h = 2r.
    Float full_thickness = 2 * thickness;
    return young_modulus * full_thickness * full_thickness * full_thickness
           / (12.0 * (1.0 - poisson_ratio * poisson_ratio));
}

void DiscreteShellBending::apply_to(geometry::SimplicialComplex& sc, Float young_modulus, Float poisson_ratio)
{
    Base::apply_to(sc);

    // thickness lives on the mesh (set by the membrane/stretch constitution);
    // bending is optional, so it reads the attribute instead of re-taking it.
    auto attr_thickness = sc.vertices().find<Float>(builtin::thickness);
    UIPC_ASSERT_THROW(attr_thickness != nullptr,
                      "DiscreteShellBending::apply_to(sc, E, nu) needs the vertex thickness "
                      "attribute; apply a membrane (stretch) constitution with thickness first, "
                      "or use apply_to(sc, bending_stiffness) for a raw value.");

    auto t_view = geometry::view(*attr_thickness);
    auto edges  = sc.edges().topo().view();

    auto bs = sc.edges().find<Float>("bending_stiffness");
    if(!bs)
    {
        bs = sc.edges().create<Float>("bending_stiffness");
    }
    auto bs_view = geometry::view(*bs);

    Float denom = 12.0 * (1.0 - poisson_ratio * poisson_ratio);
    for(SizeT i = 0; i < edges.size(); ++i)
    {
        // Average the one-sided offsets, then convert to full material thickness.
        Float radius_e = (t_view[edges[i](0)] + t_view[edges[i](1)]) * 0.5;
        Float full_thickness = 2 * radius_e;
        bs_view[i] = young_modulus * full_thickness * full_thickness * full_thickness / denom;
    }
}

U64 DiscreteShellBending::get_uid() const noexcept
{
    return DiscreteShellBendingUID;
}

Json DiscreteShellBending::default_config()
{
    return Json::object();
}
}  // namespace uipc::constitution
