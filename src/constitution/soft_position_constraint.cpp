#include <uipc/constitution/soft_position_constraint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
namespace uipc::constitution
{
constexpr U64 SoftPositionConstraintUID = 14;
REGISTER_CONSTITUTION_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = SoftPositionConstraintUID,
                           .name = "SoftPositionConstraint",
                           .type = string{builtin::Constraint}});
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
    else
        UIPC_WARN_WITH_LOCATION("Attribute `strength_ratio` on vertices already exists, which may cause ambiguity.");

    auto is_constrained = sc.vertices().find<IndexT>(builtin::is_constrained);
    if(!is_constrained)
        is_constrained = sc.vertices().create<IndexT>(builtin::is_constrained, 0);
    // NOTE: Don't fill is_constrained, if it exists, it may be filled by other constraints.
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
