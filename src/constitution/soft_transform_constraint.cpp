#include <uipc/constitution/soft_transform_constraint.h>
#include <uipc/builtin/constitution_uid_auto_register.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/builtin/constitution_type.h>
#include <uipc/common/range.h>
#include <Eigen/Geometry>

namespace uipc::constitution
{
constexpr U64 SoftTransformConstraintUID = 16;

REGISTER_CONSTITUTION_UIDS()
{
    using namespace builtin;
    list<UIDInfo> uids;
    uids.push_back(UIDInfo{.uid  = SoftTransformConstraintUID,
                           .name = "SoftTransformConstraint",
                           .type = string{builtin::Constraint}});
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

    // create constraint specified attributes
    auto constraint_strength = sc.instances().find<Vector2>("strength_ratio");
    if(!constraint_strength)
        constraint_strength =
            sc.instances().create<Vector2>("strength_ratio", strength_rate);
    else
        UIPC_WARN_WITH_LOCATION("Attribute `strength_ratio` on instances already exists, which may cause ambiguity.");

    // create is_constrained attribute
    auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
    if(!is_constrained)
        is_constrained = sc.instances().create<IndexT>(builtin::is_constrained, 0);
    // NOTE: Don't fill is_constrained, if it exists, it may be filled by other constraints.
}

Json SoftTransformConstraint::default_config()
{
    return Json::object();
}

U64 SoftTransformConstraint::get_uid() const noexcept
{
    return SoftTransformConstraintUID;
}

RotatingMotor::RotatingMotor(const Json& config) noexcept
    : m_config{config}
{
}

void RotatingMotor::apply_to(geometry::SimplicialComplex& sc,
                             Float                        strength,
                             const Vector3&               motor_rot_axis_v,
                             Float                        motor_rot_vel_v) const
{
    Base::apply_to(sc);

    sc.instances().share(builtin::aim_transform, sc.transforms());

    auto constraint_strength = sc.instances().find<Vector2>("strength_ratio");
    if(!constraint_strength)
        constraint_strength =
            sc.instances().create<Vector2>("strength_ratio", Vector2{0.0, strength});
    else
        UIPC_WARN_WITH_LOCATION("Attribute `strength_ratio` on instances already exists, which may cause ambiguity.");

    auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
    if(!is_constrained)
        is_constrained = sc.instances().create<IndexT>(builtin::is_constrained, 0);
    // NOTE: Don't fill is_constrained, if it exists, it may be filled by other constraints.


    // create motor specified attributes
    // motor_axis <Vector3>
    // motor_rot_vel <Float>

    auto motor_axis = sc.instances().find<Vector3>("motor_rot_axis");
    if(!motor_axis)
        motor_axis = sc.instances().create<Vector3>("motor_rot_axis", motor_rot_axis_v);
    else
        UIPC_WARN_WITH_LOCATION("Attribute `motor_rot_axis` on instances already exists, which may cause ambiguity.");

    auto motor_rot_vel = sc.instances().find<Float>("motor_rot_vel");
    if(!motor_rot_vel)
        motor_rot_vel = sc.instances().create<Float>("motor_rot_vel", motor_rot_vel_v);
    else
        UIPC_WARN_WITH_LOCATION("Attribute `motor_rot_vel` on instances already exists, which may cause ambiguity.");
}

Json RotatingMotor::default_config()
{
    return Json::object();
}

void RotatingMotor::animate(geometry::SimplicialComplex& sc, Float dt)
{
    auto motor_rot_axis = sc.instances().find<Vector3>("motor_rot_axis");
    UIPC_ASSERT(motor_rot_axis, "`motor_rot_axis` is not found in the simplicial complex instances, why can it happen?");
    auto motor_rot_vel = sc.instances().find<Float>("motor_rot_vel");
    UIPC_ASSERT(motor_rot_vel, "`motor_rot_vel` is not found in the simplicial complex instances, why can it happen?");

    auto motor_rot_axis_view = motor_rot_axis->view();
    auto motor_rot_vel_view  = motor_rot_vel->view();

    auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
    UIPC_ASSERT(is_constrained, "`is_constrained` is not found in the simplicial complex instances, why can it happen?");
    auto is_constrained_view = is_constrained->view();

    auto aim_transform = sc.instances().find<Matrix4x4>(builtin::aim_transform);
    UIPC_ASSERT(aim_transform, "`aim_transform` is not found in the simplicial complex instances, why can it happen?");
    auto aim_transform_view = geometry::view(*aim_transform);

    auto transform_view = sc.transforms().view();

    for(auto I : range(sc.instances().size()))
    {
        if(!is_constrained_view[I])
            continue;

        Vector3 axis  = motor_rot_axis_view[I].normalized();
        Float   omega = motor_rot_vel_view[I];
        Float   theta = omega * dt;


        Eigen::AngleAxis<Float> angle_axis(theta, axis);
        Transform               trans{transform_view[I]};

        // rotate in material space, not in world
        trans.prerotate(angle_axis);
        aim_transform_view[I] = trans.matrix();
    }
}

U64 RotatingMotor::get_uid() const noexcept
{
    return SoftTransformConstraintUID;
}

LinearMotor::LinearMotor(const Json& config) noexcept
    : m_config{config}
{
}

void LinearMotor::apply_to(geometry::SimplicialComplex& sc,
                           Float                        strength_ratio,
                           Vector3                      motor_axis_v,
                           Float                        motor_vel_v) const
{
    Base::apply_to(sc);

    sc.instances().share(builtin::aim_transform, sc.transforms());

    auto constraint_strength = sc.instances().find<Vector2>("strength_ratio");
    if(!constraint_strength)
        constraint_strength =
            sc.instances().create<Vector2>("strength_ratio", Vector2{strength_ratio, 0.0});
    else
        UIPC_WARN_WITH_LOCATION("Attribute `strength_ratio` on instances already exists, which may cause ambiguity.");

    auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
    if(!is_constrained)
        is_constrained = sc.instances().create<IndexT>(builtin::is_constrained, 0);
    // NOTE: Don't fill is_constrained, if it exists, it may be filled by other constraints.


    // create motor specified attributes
    // motor_axis <Vector3>
    // motor_vel <Float>
    auto motor_axis = sc.instances().find<Vector3>("motor_axis");
    if(!motor_axis)
        motor_axis = sc.instances().create<Vector3>("motor_axis", motor_axis_v);
    else
        UIPC_WARN_WITH_LOCATION("Attribute `motor_axis` on instances already exists, which may cause ambiguity.");

    auto motor_vel = sc.instances().find<Float>("motor_vel");
    if(!motor_vel)
        motor_vel = sc.instances().create<Float>("motor_vel", motor_vel_v);
    else
        UIPC_WARN_WITH_LOCATION("Attribute `motor_vel` on instances already exists, which may cause ambiguity.");
}

Json LinearMotor::default_config()
{
    return Json::object();
}

void LinearMotor::animate(geometry::SimplicialComplex& sc, Float dt)
{
    auto motor_axis = sc.instances().find<Vector3>("motor_axis");
    UIPC_ASSERT(motor_axis, "`motor_axis` is not found in the simplicial complex instances, why can it happen?");
    auto motor_vel = sc.instances().find<Float>("motor_vel");
    UIPC_ASSERT(motor_vel, "`motor_vel` is not found in the simplicial complex instances, why can it happen?");

    auto motor_axis_view = motor_axis->view();
    auto motor_vel_view  = motor_vel->view();

    auto is_constrained = sc.instances().find<IndexT>(builtin::is_constrained);
    UIPC_ASSERT(is_constrained, "`is_constrained` is not found in the simplicial complex instances, why can it happen?");
    auto is_constrained_view = is_constrained->view();

    auto aim_transform = sc.instances().find<Matrix4x4>(builtin::aim_transform);
    UIPC_ASSERT(aim_transform, "`aim_transform` is not found in the simplicial complex instances, why can it happen?");
    auto aim_transform_view = geometry::view(*aim_transform);

    auto transform_view = sc.transforms().view();

    for(auto I : range(sc.instances().size()))
    {
        if(!is_constrained_view[I])
            continue;

        Vector3 axis = motor_axis_view[I].normalized();
        Float   vel  = motor_vel_view[I];
        Float   d    = vel * dt;

        Transform trans{transform_view[I]};

        // move in material space, not in world
        trans.pretranslate(axis * d);

        aim_transform_view[I] = trans.matrix();
    }
}

U64 LinearMotor::get_uid() const noexcept
{
    return SoftTransformConstraintUID;
}
}  // namespace uipc::constitution
