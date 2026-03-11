#include <affine_body/constraints/affine_body_prismatic_joint_external_body_force_constraint.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>

// AffineBodyPrismaticJoint constitution UID
static constexpr uipc::U64 PrismaticJointConstitutionUID = 20;

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalBodyForceConstraint);

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_build(BuildInfo& info)
{
    require<AffineBodyDynamics>();
}

U64 AffineBodyPrismaticJointExternalBodyForceConstraint::get_uid() const noexcept
{
    return UID;
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_init(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    list<Float>    forces_list;
    list<Vector2i> body_ids_list;
    list<Vector6>  rest_tangent_list;

    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            // check constitution uid
            {
                auto constitution_uid = geo.meta().find<U64>(builtin::constitution_uid);
                UIPC_ASSERT(constitution_uid, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'constitution_uid' attribute");
                U64 uid_value = constitution_uid->view().front();
                UIPC_ASSERT(uid_value == PrismaticJointConstitutionUID,
                            "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry constitution UID mismatch, expected {}, got {}",
                            PrismaticJointConstitutionUID,
                            uid_value);
            }

            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalBodyForceConstraint: geometry must be SimplicialComplex");

            auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
            UIPC_ASSERT(geo_ids, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'geo_ids' attribute on `edges`");
            auto geo_ids_view = geo_ids->view();

            auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
            UIPC_ASSERT(inst_ids, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'inst_ids' attribute on `edges`");
            auto inst_ids_view = inst_ids->view();

            auto is_constrained = sc->edges().find<IndexT>("external_force/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_force = sc->edges().find<Float>("external_force");
            UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'external_force' attribute on `edges`");
            auto external_force_view = external_force->view();

            auto Es = sc->edges().topo().view();
            auto Ps = sc->positions().view();

            for(auto&& [i, e] : enumerate(Es))
            {
                if(is_constrained_view[i] == 0)
                    continue;

                Vector2i geo_id  = geo_ids_view[i];
                Vector2i inst_id = inst_ids_view[i];

                Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                     info.body_id(geo_id(1), inst_id(1))};
                body_ids_list.push_back(body_ids);

                auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                auto right_sc = info.body_geo(geo_slots, geo_id(1));

                Vector3 P0 = Ps[e[0]];
                Vector3 P1 = Ps[e[1]];
                Vector3 tangent = (P1 - P0).normalized();

                Transform LT{left_sc->transforms().view()[inst_id(0)]};
                Transform RT{right_sc->transforms().view()[inst_id(1)]};

                Vector6 rest_tangent;
                rest_tangent.segment<3>(0) = LT.rotation().inverse() * tangent;
                rest_tangent.segment<3>(3) = RT.rotation().inverse() * tangent;
                rest_tangent_list.push_back(rest_tangent);

                forces_list.push_back(external_force_view[i]);
            }
        });

    m_impl.h_forces.resize(forces_list.size());
    std::ranges::move(forces_list, m_impl.h_forces.begin());

    m_impl.h_body_ids.resize(body_ids_list.size());
    std::ranges::move(body_ids_list, m_impl.h_body_ids.begin());

    m_impl.h_rest_tangents.resize(rest_tangent_list.size());
    std::ranges::move(rest_tangent_list, m_impl.h_rest_tangents.begin());

    if(!m_impl.h_forces.empty())
    {
        m_impl.forces.copy_from(m_impl.h_forces);
        m_impl.body_ids.copy_from(m_impl.h_body_ids);
        m_impl.rest_tangents.copy_from(m_impl.h_rest_tangents);
    }
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_step(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    m_impl.h_forces.clear();
    m_impl.h_body_ids.clear();
    m_impl.h_rest_tangents.clear();

    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalBodyForceConstraint: geometry must be SimplicialComplex");

            auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
            UIPC_ASSERT(geo_ids, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'geo_ids' attribute on `edges`");
            auto geo_ids_view = geo_ids->view();

            auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
            UIPC_ASSERT(inst_ids, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'inst_ids' attribute on `edges`");
            auto inst_ids_view = inst_ids->view();

            auto is_constrained = sc->edges().find<IndexT>("external_force/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_force = sc->edges().find<Float>("external_force");
            UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'external_force' attribute on `edges`");
            auto external_force_view = external_force->view();

            auto Es = sc->edges().topo().view();
            auto Ps = sc->positions().view();

            for(auto&& [i, e] : enumerate(Es))
            {
                if(is_constrained_view[i] == 0)
                    continue;

                Vector2i geo_id  = geo_ids_view[i];
                Vector2i inst_id = inst_ids_view[i];

                Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                     info.body_id(geo_id(1), inst_id(1))};
                m_impl.h_body_ids.push_back(body_ids);

                auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                auto right_sc = info.body_geo(geo_slots, geo_id(1));

                Vector3 P0 = Ps[e[0]];
                Vector3 P1 = Ps[e[1]];
                Vector3 tangent = (P1 - P0).normalized();

                Transform LT{left_sc->transforms().view()[inst_id(0)]};
                Transform RT{right_sc->transforms().view()[inst_id(1)]};

                Vector6 rest_tangent;
                rest_tangent.segment<3>(0) = LT.rotation().inverse() * tangent;
                rest_tangent.segment<3>(3) = RT.rotation().inverse() * tangent;
                m_impl.h_rest_tangents.push_back(rest_tangent);

                m_impl.h_forces.push_back(external_force_view[i]);
            }
        });

    if(!m_impl.h_forces.empty())
    {
        m_impl.forces.copy_from(m_impl.h_forces);
        m_impl.body_ids.copy_from(m_impl.h_body_ids);
        m_impl.rest_tangents.copy_from(m_impl.h_rest_tangents);
    }
    else
    {
        m_impl.forces.resize(0);
        m_impl.body_ids.resize(0);
        m_impl.rest_tangents.resize(0);
    }
}

muda::CBufferView<Float> AffineBodyPrismaticJointExternalBodyForceConstraint::forces() const noexcept
{
    return m_impl.forces.view();
}

muda::CBufferView<Vector2i> AffineBodyPrismaticJointExternalBodyForceConstraint::body_ids() const noexcept
{
    return m_impl.body_ids.view();
}

muda::CBufferView<Vector6> AffineBodyPrismaticJointExternalBodyForceConstraint::rest_tangents() const noexcept
{
    return m_impl.rest_tangents.view();
}

// No energy/gradient/hessian for external forces
void AffineBodyPrismaticJointExternalBodyForceConstraint::do_report_extent(InterAffineBodyAnimator::ReportExtentInfo& info)
{
    info.energy_count(0);
    info.gradient_count(0);
    if(!info.gradient_only())
        info.hessian_count(0);
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_compute_energy(InterAffineBodyAnimator::ComputeEnergyInfo& info)
{
    // No energy computation
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_compute_gradient_hessian(InterAffineBodyAnimator::GradientHessianInfo& info)
{
    // No gradient/hessian computation
}
}  // namespace uipc::backend::cuda
