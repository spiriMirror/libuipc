#include <affine_body/constraints/affine_body_revolute_joint_external_force_constraint.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>

static constexpr uipc::U64 RevoluteJointConstitutionUID = 18;

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalForceConstraint);

void AffineBodyRevoluteJointExternalForceConstraint::do_build(BuildInfo& info)
{
    require<AffineBodyDynamics>();
    on_write_scene([this]() { write_scene(); });
}

U64 AffineBodyRevoluteJointExternalForceConstraint::get_uid() const noexcept
{
    return UID;
}

static void collect_joint_data(InterAffineBodyAnimator::FilteredInfo& info,
                               span<S<geometry::GeometrySlot>>&       geo_slots,
                               vector<Float>&                         h_torques,
                               vector<Vector2i>& h_body_ids,
                               vector<Vector12>& h_rest_positions,
                               vector<Vector6>&  h_rest_axis,
                               vector<Vector6>&  h_rest_normals,
                               vector<Float>&    h_init_angles,
                               vector<IndexT>&   h_is_constrained)
{
    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyRevoluteJointExternalForceConstraint: geometry must be SimplicialComplex");

            auto l_geo_id = sc->edges().find<IndexT>("l_geo_id");
            UIPC_ASSERT(l_geo_id, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'l_geo_id' attribute on `edges`");
            auto l_geo_id_view = l_geo_id->view();
            auto r_geo_id = sc->edges().find<IndexT>("r_geo_id");
            UIPC_ASSERT(r_geo_id, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'r_geo_id' attribute on `edges`");
            auto r_geo_id_view = r_geo_id->view();
            auto l_inst_id = sc->edges().find<IndexT>("l_inst_id");
            UIPC_ASSERT(l_inst_id, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'l_inst_id' attribute on `edges`");
            auto l_inst_id_view = l_inst_id->view();
            auto r_inst_id = sc->edges().find<IndexT>("r_inst_id");
            UIPC_ASSERT(r_inst_id, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'r_inst_id' attribute on `edges`");
            auto r_inst_id_view = r_inst_id->view();

            auto is_constrained = sc->edges().find<IndexT>("external_torque/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_torque = sc->edges().find<Float>("external_torque");
            UIPC_ASSERT(external_torque, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque' attribute on `edges`");
            auto external_torque_view = external_torque->view();

            auto init_angle = sc->edges().find<Float>("init_angle");

            auto Es = sc->edges().topo().view();
            auto Ps = sc->positions().view();

            auto l_pos0_attr = sc->edges().find<Vector3>("l_position0");
            auto l_pos1_attr = sc->edges().find<Vector3>("l_position1");
            auto r_pos0_attr = sc->edges().find<Vector3>("r_position0");
            auto r_pos1_attr = sc->edges().find<Vector3>("r_position1");
            bool use_local = l_pos0_attr && l_pos1_attr && r_pos0_attr && r_pos1_attr;

            auto toNormal = [&](const Vector3& W) -> Vector3
            {
                Vector3 ref = abs(W.dot(Vector3(1, 0, 0))) < 0.99 ?
                                  Vector3(1, 0, 0) :
                                  Vector3(0, 1, 0);

                Vector3 U = ref.cross(W).normalized();
                Vector3 V = W.cross(U).normalized();

                return V;
            };

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

                Vector3  UnitE;
                Vector12 rest_pos;
                if(use_local)
                {
                    Vector3 lp0 = LT * l_pos0_attr->view()[i];
                    Vector3 lp1 = LT * l_pos1_attr->view()[i];
                    UnitE = (lp1 - lp0).normalized();

                    rest_pos.segment<3>(0) = l_pos0_attr->view()[i];
                    rest_pos.segment<3>(3) = l_pos1_attr->view()[i];
                    rest_pos.segment<3>(6) = r_pos0_attr->view()[i];
                    rest_pos.segment<3>(9) = r_pos1_attr->view()[i];
                }
                else
                {
                    Vector3 P0  = Ps[e[0]];
                    Vector3 P1  = Ps[e[1]];
                    Vector3 mid = (P0 + P1) / 2;
                    Vector3 Dir = (P1 - P0);

                    UIPC_ASSERT(Dir.squaredNorm() > 1e-24,
                                "AffineBodyRevoluteJointExternalForceConstraint: Edge with zero length detected");

                    Vector3 HalfAxis = Dir.normalized() / 2;
                    P0               = mid - HalfAxis;
                    P1               = mid + HalfAxis;
                    UnitE            = (P1 - P0).normalized();

                    rest_pos.segment<3>(0) = LT.inverse() * P0;
                    rest_pos.segment<3>(3) = LT.inverse() * P1;
                    rest_pos.segment<3>(6) = RT.inverse() * P0;
                    rest_pos.segment<3>(9) = RT.inverse() * P1;
                }
                h_rest_positions.push_back(rest_pos);

                Vector3 normal = toNormal(UnitE);
                Vector3 vec    = normal.cross(UnitE).normalized();

                Vector6 rest_axis;
                rest_axis.segment<3>(0) = LT.rotation().inverse() * vec;
                rest_axis.segment<3>(3) = RT.rotation().inverse() * vec;
                h_rest_axis.push_back(rest_axis);

                Vector6 rest_normal;
                rest_normal.segment<3>(0) = LT.rotation().inverse() * normal;
                rest_normal.segment<3>(3) = RT.rotation().inverse() * normal;
                h_rest_normals.push_back(rest_normal);

                h_torques.push_back(external_torque_view[i]);
                h_init_angles.push_back(init_angle ? init_angle->view()[i] : Float{0});
            }
        });
}

void AffineBodyRevoluteJointExternalForceConstraint::do_init(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    {
        info.for_each(geo_slots,
                      [&](const InterAffineBodyConstitutionManager::ForEachInfo& I,
                          geometry::Geometry& geo)
                      {
                          auto constitution_uid =
                              geo.meta().find<U64>(builtin::constitution_uid);
                          UIPC_ASSERT(constitution_uid, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'constitution_uid' attribute");
                          U64 uid_value = constitution_uid->view().front();
                          UIPC_ASSERT(uid_value == RevoluteJointConstitutionUID,
                                      "AffineBodyRevoluteJointExternalForceConstraint: Geometry constitution UID mismatch, expected {}, got {}",
                                      RevoluteJointConstitutionUID,
                                      uid_value);
                      });
    }

    collect_joint_data(info,
                       geo_slots,
                       m_impl.h_torques,
                       m_impl.h_body_ids,
                       m_impl.h_rest_positions,
                       m_impl.h_rest_axis,
                       m_impl.h_rest_normals,
                       m_impl.h_init_angles,
                       m_impl.h_is_constrained);

    SizeT N = m_impl.h_torques.size();
    m_impl.h_current_angles.resize(N, 0.0);

    if(N > 0)
    {
        m_impl.torques.copy_from(m_impl.h_torques);
        m_impl.body_ids.copy_from(m_impl.h_body_ids);
        m_impl.rest_positions.copy_from(m_impl.h_rest_positions);
        m_impl.rest_axis.copy_from(m_impl.h_rest_axis);
        m_impl.rest_normals.copy_from(m_impl.h_rest_normals);
        m_impl.init_angles.copy_from(m_impl.h_init_angles);
        m_impl.is_constrained.copy_from(m_impl.h_is_constrained);
        m_impl.current_angles.copy_from(m_impl.h_init_angles);
    }
}

void AffineBodyRevoluteJointExternalForceConstraint::do_step(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    // Only update is_constrained and external_torque (structural data unchanged from do_init)
    SizeT offset = 0;
    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyRevoluteJointExternalForceConstraint: geometry must be SimplicialComplex");

            auto is_constrained = sc->edges().find<IndexT>("external_torque/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_torque = sc->edges().find<Float>("external_torque");
            UIPC_ASSERT(external_torque, "AffineBodyRevoluteJointExternalForceConstraint: Geometry must have 'external_torque' attribute on `edges`");
            auto external_torque_view = external_torque->view();

            auto Es = sc->edges().topo().view();
            for(auto&& [i, e] : enumerate(Es))
            {
                m_impl.h_is_constrained[offset] = is_constrained_view[i];
                m_impl.h_torques[offset]        = external_torque_view[i];
                ++offset;
            }
        });

    SizeT N = m_impl.h_torques.size();
    if(N > 0)
    {
        m_impl.is_constrained.copy_from(m_impl.h_is_constrained);
        m_impl.torques.copy_from(m_impl.h_torques);
    }
}

void AffineBodyRevoluteJointExternalForceConstraint::write_scene()
{
    auto geo_slots = world().scene().geometries();

    m_impl.current_angles.copy_to(m_impl.h_current_angles);

    IndexT offset = 0;
    this->for_each(geo_slots,
                   [&](geometry::Geometry& geo)
                   {
                       auto sc = geo.as<geometry::SimplicialComplex>();
                       if(!sc)
                           return;

                       auto is_constrained =
                           sc->edges().find<IndexT>("external_torque/is_constrained");
                       if(!is_constrained)
                           return;
                       auto is_constrained_view = is_constrained->view();

                       auto angle = sc->edges().find<Float>("angle");
                       if(!angle)
                           angle = sc->edges().create<Float>("angle", 0.0);
                       auto angle_view = view(*angle);

                       for(SizeT i = 0; i < is_constrained_view.size(); ++i)
                       {
                           if(offset < m_impl.h_current_angles.size())
                           {
                               if(is_constrained_view[i] != 0)
                                   angle_view[i] = m_impl.h_current_angles[offset];
                               ++offset;
                           }
                       }
                   });
}

muda::CBufferView<Float> AffineBodyRevoluteJointExternalForceConstraint::torques() const noexcept
{
    return m_impl.torques.view();
}

muda::CBufferView<Vector2i> AffineBodyRevoluteJointExternalForceConstraint::body_ids() const noexcept
{
    return m_impl.body_ids.view();
}

muda::CBufferView<Vector12> AffineBodyRevoluteJointExternalForceConstraint::rest_positions() const noexcept
{
    return m_impl.rest_positions.view();
}

muda::CBufferView<Vector6> AffineBodyRevoluteJointExternalForceConstraint::rest_axis() const noexcept
{
    return m_impl.rest_axis.view();
}

muda::CBufferView<Vector6> AffineBodyRevoluteJointExternalForceConstraint::rest_normals() const noexcept
{
    return m_impl.rest_normals.view();
}


muda::CBufferView<Float> AffineBodyRevoluteJointExternalForceConstraint::init_angles() const noexcept
{
    return m_impl.init_angles.view();
}

muda::DeviceBuffer<Float>& AffineBodyRevoluteJointExternalForceConstraint::current_angles() noexcept
{
    return m_impl.current_angles;
}

muda::CBufferView<IndexT> AffineBodyRevoluteJointExternalForceConstraint::constrained_flags() const noexcept
{
    return m_impl.is_constrained.view();
}

void AffineBodyRevoluteJointExternalForceConstraint::do_report_extent(
    InterAffineBodyAnimator::ReportExtentInfo& info)
{
    info.energy_count(0);
    info.gradient_count(0);
    if(!info.gradient_only())
        info.hessian_count(0);
}

void AffineBodyRevoluteJointExternalForceConstraint::do_compute_energy(
    InterAffineBodyAnimator::ComputeEnergyInfo& info)
{
}

void AffineBodyRevoluteJointExternalForceConstraint::do_compute_gradient_hessian(
    InterAffineBodyAnimator::GradientHessianInfo& info)
{
}
}  // namespace uipc::backend::cuda
