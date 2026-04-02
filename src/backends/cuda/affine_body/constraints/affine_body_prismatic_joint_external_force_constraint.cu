#include <affine_body/constraints/affine_body_prismatic_joint_external_force_constraint.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>

static constexpr uipc::U64 PrismaticJointConstitutionUID = 20;

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyPrismaticJointExternalForceConstraint);

void AffineBodyPrismaticJointExternalForceConstraint::do_build(BuildInfo& info)
{
    require<AffineBodyDynamics>();
    on_write_scene([this]() { write_scene(); });
}

U64 AffineBodyPrismaticJointExternalForceConstraint::get_uid() const noexcept
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
                UIPC_ASSERT(constitution_uid, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'constitution_uid' attribute");
                U64 uid_value = constitution_uid->view().front();
                UIPC_ASSERT(uid_value == PrismaticJointConstitutionUID,
                            "AffineBodyPrismaticJointExternalForceConstraint: Geometry constitution UID mismatch, expected {}, got {}",
                            PrismaticJointConstitutionUID,
                            uid_value);
            }

            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalForceConstraint: geometry must be SimplicialComplex");

            auto l_geo_id = sc->edges().find<IndexT>("l_geo_id");
            UIPC_ASSERT(l_geo_id, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'l_geo_id' attribute on `edges`");
            auto l_geo_id_view = l_geo_id->view();
            auto r_geo_id = sc->edges().find<IndexT>("r_geo_id");
            UIPC_ASSERT(r_geo_id, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'r_geo_id' attribute on `edges`");
            auto r_geo_id_view = r_geo_id->view();
            auto l_inst_id = sc->edges().find<IndexT>("l_inst_id");
            UIPC_ASSERT(l_inst_id, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'l_inst_id' attribute on `edges`");
            auto l_inst_id_view = l_inst_id->view();
            auto r_inst_id = sc->edges().find<IndexT>("r_inst_id");
            UIPC_ASSERT(r_inst_id, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'r_inst_id' attribute on `edges`");
            auto r_inst_id_view = r_inst_id->view();

            auto is_constrained = sc->edges().find<IndexT>("external_force/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_force = sc->edges().find<Float>("external_force");
            UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force' attribute on `edges`");
            auto external_force_view = external_force->view();

            auto init_distance = sc->edges().find<Float>("init_distance");

            auto Es = sc->edges().topo().view();
            auto Ps = sc->positions().view();

            auto l_pos0_attr = sc->edges().find<Vector3>("l_position0");
            auto l_pos1_attr = sc->edges().find<Vector3>("l_position1");
            auto r_pos0_attr = sc->edges().find<Vector3>("r_position0");
            auto r_pos1_attr = sc->edges().find<Vector3>("r_position1");
            bool use_local = l_pos0_attr && l_pos1_attr && r_pos0_attr && r_pos1_attr;

            for(auto&& [i, e] : enumerate(Es))
            {
                h_is_constrained.push_back(is_constrained_view[i]);

                IndexT l_gid = l_geo_id_view[i];
                IndexT r_gid = r_geo_id_view[i];
                IndexT l_iid = l_inst_id_view[i];
                IndexT r_iid = r_inst_id_view[i];

                Vector2i body_ids = {info.body_id(l_gid, l_iid),
                                     info.body_id(r_gid, r_iid)};
                h_body_ids.push_back(body_ids);

                auto left_sc  = info.body_geo(geo_slots, l_gid);
                auto right_sc = info.body_geo(geo_slots, r_gid);

                Transform LT{left_sc->transforms().view()[l_iid]};
                Transform RT{right_sc->transforms().view()[r_iid]};

                Vector3 tangent;
                Vector6 rest_position;
                if(use_local)
                {
                    Vector3 lp0 = LT * l_pos0_attr->view()[i];
                    Vector3 lp1 = LT * l_pos1_attr->view()[i];
                    tangent = (lp1 - lp0).normalized();
                    rest_position.segment<3>(0) = l_pos0_attr->view()[i];
                    rest_position.segment<3>(3) = r_pos0_attr->view()[i];
                }
                else
                {
                    Vector3 P0      = Ps[e[0]];
                    Vector3 P1      = Ps[e[1]];
                    tangent = (P1 - P0).normalized();
                    rest_position.segment<3>(0) = LT.inverse() * P0;
                    rest_position.segment<3>(3) = RT.inverse() * P0;
                }

                Vector6 rest_tangent;
                rest_tangent.segment<3>(0) = LT.rotation().inverse() * tangent;
                rest_tangent.segment<3>(3) = RT.rotation().inverse() * tangent;
                h_rest_tangents.push_back(rest_tangent);

                h_rest_positions.push_back(rest_position);

                h_forces.push_back(external_force_view[i]);
                h_init_distances.push_back(init_distance ? init_distance->view()[i] :
                                                           Float{0});
            }
        });
}

void AffineBodyPrismaticJointExternalForceConstraint::do_init(InterAffineBodyAnimator::FilteredInfo& info)
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

void AffineBodyPrismaticJointExternalForceConstraint::do_step(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    // Only update is_constrained and external_force (structural data unchanged from do_init)
    SizeT offset = 0;
    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyPrismaticJointExternalForceConstraint: geometry must be SimplicialComplex");

            auto is_constrained = sc->edges().find<IndexT>("external_force/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_force = sc->edges().find<Float>("external_force");
            UIPC_ASSERT(external_force, "AffineBodyPrismaticJointExternalForceConstraint: Geometry must have 'external_force' attribute on `edges`");
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

void AffineBodyPrismaticJointExternalForceConstraint::write_scene()
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

muda::CBufferView<Float> AffineBodyPrismaticJointExternalForceConstraint::forces() const noexcept
{
    return m_impl.forces.view();
}

muda::CBufferView<Vector2i> AffineBodyPrismaticJointExternalForceConstraint::body_ids() const noexcept
{
    return m_impl.body_ids.view();
}

muda::CBufferView<Vector6> AffineBodyPrismaticJointExternalForceConstraint::rest_tangents() const noexcept
{
    return m_impl.rest_tangents.view();
}

muda::CBufferView<Vector6> AffineBodyPrismaticJointExternalForceConstraint::rest_positions() const noexcept
{
    return m_impl.rest_positions.view();
}

muda::CBufferView<Float> AffineBodyPrismaticJointExternalForceConstraint::init_distances() const noexcept
{
    return m_impl.init_distances.view();
}

muda::DeviceBuffer<Float>& AffineBodyPrismaticJointExternalForceConstraint::current_distances() noexcept
{
    return m_impl.current_distances;
}

muda::CBufferView<IndexT> AffineBodyPrismaticJointExternalForceConstraint::constrained_flags() const noexcept
{
    return m_impl.is_constrained.view();
}

void AffineBodyPrismaticJointExternalForceConstraint::do_report_extent(
    InterAffineBodyAnimator::ReportExtentInfo& info)
{
    info.energy_count(0);
    info.gradient_count(0);
    if(!info.gradient_only())
        info.hessian_count(0);
}

void AffineBodyPrismaticJointExternalForceConstraint::do_compute_energy(
    InterAffineBodyAnimator::ComputeEnergyInfo& info)
{
}

void AffineBodyPrismaticJointExternalForceConstraint::do_compute_gradient_hessian(
    InterAffineBodyAnimator::GradientHessianInfo& info)
{
}
}  // namespace uipc::backend::cuda
