#include <uipc/constitution/soft_transform_constraint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
namespace uipc::constitution
{
constexpr U64 SoftTransformConstraintUID = 15;
REGISTER_CONSTITUTION_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{
        .uid  = SoftTransformConstraintUID,
        .name = "Constraint::SoftTransformConstraint",
    });
    return uids;
};

SoftTransformConstraint::SoftTransformConstraint(const Json& config) noexcept
    : m_config(config)
{
}

void SoftTransformConstraint::apply_to(geometry::SimplicialComplex& sc,
                                       const Vector2& strength_rate) const
{
    Base::apply_to(sc);

    sc.instances().share(builtin::aim_transform, sc.transforms());

    constexpr std::string_view strength_ratio = "strength_ratio";

    auto constraint_strength = sc.instances().find<Vector2>(strength_ratio);
    if(!constraint_strength)
        constraint_strength = sc.instances().create<Vector2>(strength_ratio, strength_rate);

    auto strength_view = geometry::view(*constraint_strength);
    std::ranges::fill(strength_view, strength_rate);

    auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);

    if(!is_constrained)
        is_constrained = sc.instances().create<IndexT>(builtin::is_constrained, 0);
}

Json SoftTransformConstraint::default_config()
{
    return Json::object();
}

U64 SoftTransformConstraint::get_uid() const noexcept
{
    return SoftTransformConstraintUID;
}
}  // namespace uipc::constitution
