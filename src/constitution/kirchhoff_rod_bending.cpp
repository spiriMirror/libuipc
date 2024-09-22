#include <uipc/constitution/kirchhoff_rod_bending.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
namespace uipc::constitution
{
constexpr U64 KirchhoffRodBendingUID = 15;

REGISTER_CONSTITUTION_UIDS()
{
    list<builtin::UIDInfo> uid_infos;
    builtin::UIDInfo       info;
    info.uid  = KirchhoffRodBendingUID;
    info.name = "KirchhoffRodBending";
    info.type = string{builtin::FiniteElement};
    uid_infos.push_back(info);
    return uid_infos;
}

KirchhoffRodBending::KirchhoffRodBending(const Json& json)
    : m_config{json}
{
}

void KirchhoffRodBending::apply_to(geometry::SimplicialComplex& sc, Float E)
{
    Base::apply_to(sc);
    auto bs = sc.vertices().find<Float>("bending_stiffness");
    if(!bs)
    {
        bs = sc.vertices().create<Float>("bending_stiffness");
    }
    auto bs_view = geometry::view(*bs);
    std::ranges::fill(bs_view, E);
}

U64 KirchhoffRodBending::get_uid() const noexcept
{
    return KirchhoffRodBendingUID;
}

Json KirchhoffRodBending::default_config()
{
    return Json::object();
}
}  // namespace uipc::constitution
