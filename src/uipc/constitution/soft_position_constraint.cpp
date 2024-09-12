#include <uipc/constitution/soft_position_constraint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
namespace uipc::constitution
{
constexpr U64 SoftPositionConstraintUID = 14;
REGISTER_CONSTITUTION_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{
        .uid  = SoftPositionConstraintUID,
        .name = "Constraint::SoftPositionConstraint",
    });
    return uids;
};

SoftPositionConstraint::SoftPositionConstraint(const Json& config) noexcept
    : m_config(config)
{
}

void SoftPositionConstraint::apply_to(geometry::SimplicialComplex& sc, Float strength_rate) const
{
    Base::apply_to(sc);

    sc.vertices().share(builtin::aim_position, sc.positions());

    constexpr std::string_view strength_ratio = "strength_ratio";

    auto constraint_strength = sc.vertices().find<Float>(strength_ratio);
    if(!constraint_strength)
        constraint_strength = sc.vertices().create<Float>(strength_ratio, strength_rate);

    auto strength_view = geometry::view(*constraint_strength);
    std::ranges::fill(strength_view, strength_rate);

    auto is_constrained = sc.vertices().find<IndexT>(builtin::is_constrained);

    if(!is_constrained)
        is_constrained = sc.vertices().create<IndexT>(builtin::is_constrained, 0);
}

Json SoftPositionConstraint::default_config()
{
    return Json::object();
}

U64 SoftPositionConstraint::get_uid() const noexcept
{
    return SoftPositionConstraintUID;
}
}  // namespace uipc::constitution
