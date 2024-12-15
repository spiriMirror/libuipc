#include <uipc/constitution/discrete_shell_bending.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_auto_register.h>

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

void DiscreteShellBending::apply_to(geometry::SimplicialComplex& sc, Float E)
{
    Base::apply_to(sc);
    auto bs = sc.edges().find<Float>("bending_stiffness");
    if(!bs)
    {
        bs = sc.edges().create<Float>("bending_stiffness");
    }
    auto bs_view = geometry::view(*bs);
    std::ranges::fill(bs_view, E);
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
