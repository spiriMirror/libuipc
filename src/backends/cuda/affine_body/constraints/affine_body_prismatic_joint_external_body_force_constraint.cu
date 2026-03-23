#include <affine_body/constraints/affine_body_prismatic_joint_external_body_force_constraint.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>

static constexpr uipc::U64 PrismaticJointConstitutionUID = 20;

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalBodyForceConstraint);

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_build(BuildInfo& info)
{
    require<AffineBodyDynamics>();
    on_write_scene([this]() { write_scene(); });
}

U64 AffineBodyPrismaticJointExternalBodyForceConstraint::get_uid() const noexcept
{
    return UID;
}

static void collect_prismatic_data(InterAffineBodyAnimator::FilteredInfo& info,
                                   span<S<geometry::GeometrySlot>>& geo_slots,
                                   vector<Float>&                   h_forces,
                                   vector<Vector2i>&                h_body_ids,
                                   vector<Vector6>& h_rest_tangents,
                                   vector<Vector6>& h_rest_positions,
                                   vector<Float>&   h_init_distances,
                                   vector<IndexT>&  h_is_constrained,
                                   bool             check_uid = false)
{
    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            if(check_uid)
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

            auto init_distance = sc->edges().find<Float>("init_distance");

            auto Es = sc->edges().topo().view();
            auto Ps = sc->positions().view();

            for(auto&& [i, e] : enumerate(Es))
            {
                h_is_constrained.push_back(is_constrained_view[i]);

                Vector2i geo_id  = geo_ids_view[i];
                Vector2i inst_id = inst_ids_view[i];

                Vector2i body_ids = {info.body_id(geo_id(0), inst_id(0)),
                                     info.body_id(geo_id(1), inst_id(1))};
                h_body_ids.push_back(body_ids);

                auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                auto right_sc = info.body_geo(geo_slots, geo_id(1));

                Vector3 P0      = Ps[e[0]];
                Vector3 P1      = Ps[e[1]];
                Vector3 tangent = (P1 - P0).normalized();

                Transform LT{left_sc->transforms().view()[inst_id(0)]};
                Transform RT{right_sc->transforms().view()[inst_id(1)]};

                Vector6 rest_tangent;
                rest_tangent.segment<3>(0) = LT.rotation().inverse() * tangent;
                rest_tangent.segment<3>(3) = RT.rotation().inverse() * tangent;
                h_rest_tangents.push_back(rest_tangent);

                Vector6 rest_position;
                rest_position.segment<3>(0) = LT.inverse() * P0;
                rest_position.segment<3>(3) = RT.inverse() * P0;
                h_rest_positions.push_back(rest_position);

                h_forces.push_back(external_force_view[i]);
                h_init_distances.push_back(init_distance ? init_distance->view()[i] :
                                                           Float{0});
            }
        });
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_init(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    collect_prismatic_data(info,
                           geo_slots,
                           m_impl.h_forces,
                           m_impl.h_body_ids,
                           m_impl.h_rest_tangents,
                           m_impl.h_rest_positions,
                           m_impl.h_init_distances,
                           m_impl.h_is_constrained,
                           true);

    SizeT N = m_impl.h_forces.size();
    m_impl.h_current_distances.resize(N, 0.0);

    if(N > 0)
    {
        m_impl.forces.copy_from(m_impl.h_forces);
        m_impl.body_ids.copy_from(m_impl.h_body_ids);
        m_impl.rest_tangents.copy_from(m_impl.h_rest_tangents);
        m_impl.rest_positions.copy_from(m_impl.h_rest_positions);
        m_impl.init_distances.copy_from(m_impl.h_init_distances);
        m_impl.is_constrained.copy_from(m_impl.h_is_constrained);
        m_impl.current_distances.resize(N, 0.0);
    }
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_step(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    // Only update is_constrained and external_force (structural data unchanged from do_init)
    SizeT offset = 0;
    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalBodyForceConstraint: geometry must be SimplicialComplex");

            auto is_constrained = sc->edges().find<IndexT>("external_force/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_force = sc->edges().find<Float>("external_force");
            UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalBodyForceConstraint: Geometry must have 'external_force' attribute on `edges`");
            auto external_force_view = external_force->view();

            auto Es = sc->edges().topo().view();
            for(auto&& [i, e] : enumerate(Es))
            {
                m_impl.h_is_constrained[offset] = is_constrained_view[i];
                m_impl.h_forces[offset]         = external_force_view[i];
                ++offset;
            }
        });

    SizeT N = m_impl.h_forces.size();
    if(N > 0)
    {
        m_impl.is_constrained.copy_from(m_impl.h_is_constrained);
        m_impl.forces.copy_from(m_impl.h_forces);
    }
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::write_scene()
{
    auto geo_slots = world().scene().geometries();

    m_impl.current_distances.copy_to(m_impl.h_current_distances);

    IndexT offset = 0;
    this->for_each(geo_slots,
                   [&](geometry::Geometry& geo)
                   {
                       auto sc = geo.as<geometry::SimplicialComplex>();
                       if(!sc)
                           return;

                       auto is_constrained =
                           sc->edges().find<IndexT>("external_force/is_constrained");
                       if(!is_constrained)
                           return;
                       auto is_constrained_view = is_constrained->view();

                       auto distance = sc->edges().find<Float>("distance");
                       if(!distance)
                           distance = sc->edges().create<Float>("distance", 0.0);
                       auto distance_view = view(*distance);

                       for(SizeT i = 0; i < is_constrained_view.size(); ++i)
                       {
                           if(offset < m_impl.h_current_distances.size())
                           {
                               if(is_constrained_view[i] != 0)
                                   distance_view[i] = m_impl.h_current_distances[offset];
                               ++offset;
                           }
                       }
                   });
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

muda::CBufferView<Vector6> AffineBodyPrismaticJointExternalBodyForceConstraint::rest_positions() const noexcept
{
    return m_impl.rest_positions.view();
}

muda::CBufferView<Float> AffineBodyPrismaticJointExternalBodyForceConstraint::init_distances() const noexcept
{
    return m_impl.init_distances.view();
}

muda::DeviceBuffer<Float>& AffineBodyPrismaticJointExternalBodyForceConstraint::current_distances() noexcept
{
    return m_impl.current_distances;
}

muda::CBufferView<IndexT> AffineBodyPrismaticJointExternalBodyForceConstraint::constrained_flags() const noexcept
{
    return m_impl.is_constrained.view();
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_report_extent(
    InterAffineBodyAnimator::ReportExtentInfo& info)
{
    info.energy_count(0);
    info.gradient_count(0);
    if(!info.gradient_only())
        info.hessian_count(0);
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_compute_energy(
    InterAffineBodyAnimator::ComputeEnergyInfo& info)
{
}

void AffineBodyPrismaticJointExternalBodyForceConstraint::do_compute_gradient_hessian(
    InterAffineBodyAnimator::GradientHessianInfo& info)
{
}
}  // namespace uipc::backend::cuda
