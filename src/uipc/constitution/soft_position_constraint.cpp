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

void SoftPositionConstraint::apply_to(geometry::SimplicialComplex& sc,
                                      Float strength_rate) const
{
    Base::apply_to(sc);

    auto ins_is_fixed  = sc.instances().find<IndexT>(builtin::is_fixed);
    auto vert_is_fixed = sc.vertices().find<IndexT>(builtin::is_fixed);

    if constexpr(uipc::RUNTIME_CHECK)
    {
        auto both = ins_is_fixed && vert_is_fixed;
        UIPC_ASSERT(!both, "Animation: SimplicialComplex has both fixed vertices and instances, which is ambiguous.");

        auto none = !ins_is_fixed && !vert_is_fixed;
        UIPC_ASSERT(!none, "Animation: SimplicialComplex has neither fixed vertices nor instances, which is meaningless for animation.");
    }

    sc.vertices().share(builtin::aim_position, sc.positions());

    auto constraint_strength = sc.vertices().find<Float>("constraint_strength");
    if(!constraint_strength)
        constraint_strength =
            sc.vertices().create<Float>("constraint_strength", strength_rate);

    auto strength_view = geometry::view(*constraint_strength);
    std::ranges::fill(strength_view, strength_rate);
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