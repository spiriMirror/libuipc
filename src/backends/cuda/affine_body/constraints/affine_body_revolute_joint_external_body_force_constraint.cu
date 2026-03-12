#include <affine_body/constraints/affine_body_revolute_joint_external_body_force_constraint.h>
#include <affine_body/inter_affine_body_constraint.h>
#include <affine_body/affine_body_dynamics.h>
#include <uipc/common/enumerate.h>
#include <uipc/builtin/attribute_name.h>

static constexpr uipc::U64 RevoluteJointConstitutionUID = 18;

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(AffineBodyRevoluteJointExternalBodyForceConstraint);

void AffineBodyRevoluteJointExternalBodyForceConstraint::do_build(BuildInfo& info)
{
    require<AffineBodyDynamics>();
    on_write_scene([this]() { write_scene(); });
}

U64 AffineBodyRevoluteJointExternalBodyForceConstraint::get_uid() const noexcept
{
    return UID;
}

static void collect_joint_data(InterAffineBodyAnimator::FilteredInfo& info,
                               auto&                                  geo_slots,
                               vector<Float>&                         h_torques,
                               vector<Vector2i>& h_body_ids,
                               vector<Vector12>& h_rest_positions,
                               vector<Float>&    h_init_angles)
{
    info.for_each(
        geo_slots,
        [&](const InterAffineBodyConstitutionManager::ForEachInfo& I, geometry::Geometry& geo)
        {
            auto sc = geo.as<geometry::SimplicialComplex>();
            UIPC_ASSERT(sc, "AffineBodyRevoluteJointExternalBodyForceConstraint: geometry must be SimplicialComplex");

            auto geo_ids = sc->edges().find<Vector2i>("geo_ids");
            UIPC_ASSERT(geo_ids, "AffineBodyRevoluteJointExternalBodyForceConstraint: Geometry must have 'geo_ids' attribute on `edges`");
            auto geo_ids_view = geo_ids->view();

            auto inst_ids = sc->edges().find<Vector2i>("inst_ids");
            UIPC_ASSERT(inst_ids, "AffineBodyRevoluteJointExternalBodyForceConstraint: Geometry must have 'inst_ids' attribute on `edges`");
            auto inst_ids_view = inst_ids->view();

            auto is_constrained = sc->edges().find<IndexT>("external_torque/is_constrained");
            UIPC_ASSERT(is_constrained, "AffineBodyRevoluteJointExternalBodyForceConstraint: Geometry must have 'external_torque/is_constrained' attribute on `edges`");
            auto is_constrained_view = is_constrained->view();

            auto external_torque = sc->edges().find<Float>("external_torque");
            UIPC_ASSERT(external_torque, "AffineBodyRevoluteJointExternalBodyForceConstraint: Geometry must have 'external_torque' attribute on `edges`");
            auto external_torque_view = external_torque->view();

            auto init_angle = sc->edges().find<Float>("init_angle");

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
                h_body_ids.push_back(body_ids);

                auto left_sc  = info.body_geo(geo_slots, geo_id(0));
                auto right_sc = info.body_geo(geo_slots, geo_id(1));

                Vector3 P0  = Ps[e[0]];
                Vector3 P1  = Ps[e[1]];
                Vector3 mid = (P0 + P1) / 2;
                Vector3 Dir = (P1 - P0);

                UIPC_ASSERT(Dir.squaredNorm() > 1e-24,
                            "AffineBodyRevoluteJointExternalBodyForceConstraint: Edge with zero length detected");

                Vector3 HalfAxis = Dir.normalized() / 2;
                P0               = mid - HalfAxis;
                P1               = mid + HalfAxis;

                Transform LT{left_sc->transforms().view()[inst_id(0)]};
                Transform RT{right_sc->transforms().view()[inst_id(1)]};

                Vector12 rest_pos;
                rest_pos.segment<3>(0) = LT.inverse() * P0;
                rest_pos.segment<3>(3) = LT.inverse() * P1;
                rest_pos.segment<3>(6) = RT.inverse() * P0;
                rest_pos.segment<3>(9) = RT.inverse() * P1;
                h_rest_positions.push_back(rest_pos);

                h_torques.push_back(external_torque_view[i]);
                h_init_angles.push_back(init_angle ? init_angle->view()[i] : Float{0});
            }
        });
}

void AffineBodyRevoluteJointExternalBodyForceConstraint::do_init(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    {
        info.for_each(geo_slots,
                      [&](const InterAffineBodyConstitutionManager::ForEachInfo& I,
                          geometry::Geometry& geo)
                      {
                          auto constitution_uid =
                              geo.meta().find<U64>(builtin::constitution_uid);
                          UIPC_ASSERT(constitution_uid, "AffineBodyRevoluteJointExternalBodyForceConstraint: Geometry must have 'constitution_uid' attribute");
                          U64 uid_value = constitution_uid->view().front();
                          UIPC_ASSERT(uid_value == RevoluteJointConstitutionUID,
                                      "AffineBodyRevoluteJointExternalBodyForceConstraint: Geometry constitution UID mismatch, expected {}, got {}",
                                      RevoluteJointConstitutionUID,
                                      uid_value);
                      });
    }

    collect_joint_data(info,
                       geo_slots,
                       m_impl.h_torques,
                       m_impl.h_body_ids,
                       m_impl.h_rest_positions,
                       m_impl.h_init_angles);

    SizeT N = m_impl.h_torques.size();
    m_impl.h_current_angles.resize(N, 0.0);

    if(N > 0)
    {
        m_impl.torques.copy_from(m_impl.h_torques);
        m_impl.body_ids.copy_from(m_impl.h_body_ids);
        m_impl.rest_positions.copy_from(m_impl.h_rest_positions);
        m_impl.init_angles.copy_from(m_impl.h_init_angles);
        m_impl.current_angles.resize(N, 0.0);
    }
}

void AffineBodyRevoluteJointExternalBodyForceConstraint::do_step(InterAffineBodyAnimator::FilteredInfo& info)
{
    auto geo_slots = world().scene().geometries();

    m_impl.h_torques.clear();
    m_impl.h_body_ids.clear();
    m_impl.h_rest_positions.clear();
    m_impl.h_init_angles.clear();

    collect_joint_data(info,
                       geo_slots,
                       m_impl.h_torques,
                       m_impl.h_body_ids,
                       m_impl.h_rest_positions,
                       m_impl.h_init_angles);

    SizeT N = m_impl.h_torques.size();
    m_impl.h_current_angles.resize(N, 0.0);

    if(N > 0)
    {
        m_impl.torques.copy_from(m_impl.h_torques);
        m_impl.body_ids.copy_from(m_impl.h_body_ids);
        m_impl.rest_positions.copy_from(m_impl.h_rest_positions);
        m_impl.init_angles.copy_from(m_impl.h_init_angles);
        m_impl.current_angles.resize(N);
    }
    else
    {
        m_impl.torques.resize(0);
        m_impl.body_ids.resize(0);
        m_impl.rest_positions.resize(0);
        m_impl.init_angles.resize(0);
        m_impl.current_angles.resize(0);
    }
}

void AffineBodyRevoluteJointExternalBodyForceConstraint::write_scene()
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
                           if(is_constrained_view[i] == 0)
                               continue;
                           if(offset < m_impl.h_current_angles.size())
                               angle_view[i] = m_impl.h_current_angles[offset++];
                       }
                   });
}

muda::CBufferView<Float> AffineBodyRevoluteJointExternalBodyForceConstraint::torques() const noexcept
{
    return m_impl.torques.view();
}

muda::CBufferView<Vector2i> AffineBodyRevoluteJointExternalBodyForceConstraint::body_ids() const noexcept
{
    return m_impl.body_ids.view();
}

muda::CBufferView<Vector12> AffineBodyRevoluteJointExternalBodyForceConstraint::rest_positions() const noexcept
{
    return m_impl.rest_positions.view();
}

muda::CBufferView<Float> AffineBodyRevoluteJointExternalBodyForceConstraint::init_angles() const noexcept
{
    return m_impl.init_angles.view();
}

muda::DeviceBuffer<Float>& AffineBodyRevoluteJointExternalBodyForceConstraint::current_angles() noexcept
{
    return m_impl.current_angles;
}

void AffineBodyRevoluteJointExternalBodyForceConstraint::do_report_extent(
    InterAffineBodyAnimator::ReportExtentInfo& info)
{
    info.energy_count(0);
    info.gradient_count(0);
    if(!info.gradient_only())
        info.hessian_count(0);
}

void AffineBodyRevoluteJointExternalBodyForceConstraint::do_compute_energy(
    InterAffineBodyAnimator::ComputeEnergyInfo& info)
{
}

void AffineBodyRevoluteJointExternalBodyForceConstraint::do_compute_gradient_hessian(
    InterAffineBodyAnimator::GradientHessianInfo& info)
{
}
}  // namespace uipc::backend::cuda
