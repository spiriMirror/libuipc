#include <affine_body/affine_body_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
void AffineBodyExtraConstitution::do_build()
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();

    auto uids     = world().scene().constitution_tabular().uids();
    auto this_uid = uid();
    if(!std::ranges::binary_search(uids, this_uid))
    {
        throw SimSystemException(
            fmt::format("Extra Constitution UID ({}) not found in the constitution tabular",
                        this_uid));
    }

    BuildInfo this_info;
    do_build(this_info);

    m_impl.affine_body_dynamics->add_extra_constitution(this);
}

U64 AffineBodyExtraConstitution::uid() const noexcept
{
    return get_uid();
}

span<const AffineBodyDynamics::GeoInfo> AffineBodyExtraConstitution::geo_infos() const noexcept
{
    return m_impl.geo_infos;
}

void AffineBodyExtraConstitution::init()
{
    m_impl.init(uid(), world());

    // let the subclass do the rest of the initialization
    FilteredInfo info{&m_impl};
    do_init(info);
}

void AffineBodyExtraConstitution::step()
{
    FilteredInfo info{&m_impl};
    do_step(info);
}

void AffineBodyExtraConstitution::compute_energy(AffineBodyDynamics::Impl& abd_impl,
                                                  Float                     dt,
                                                  muda::BufferView<Float>   energies)
{
    ComputeEnergyInfo info{&abd_impl, dt, energies};
    do_compute_energy(info);
}

void AffineBodyExtraConstitution::compute_gradient_hessian(AffineBodyDynamics::Impl&     abd_impl,
                                                            Float                         dt,
                                                            muda::BufferView<Vector12>    gradients,
                                                            muda::BufferView<Matrix12x12> hessians)
{
    ComputeGradientHessianInfo info{&abd_impl, dt, gradients, hessians};
    do_compute_gradient_hessian(info);
}

void AffineBodyExtraConstitution::Impl::init(U64 uid, backend::WorldVisitor& world)
{
    using ForEachInfo = AffineBodyDynamics::ForEachInfo;

    // 1) Find the geometry slots that have the extra constitution uids containing the given uid
    auto& abd_geo_infos = affine_body_dynamics->m_impl.geo_infos;
    auto  geo_slots     = world.scene().geometries();

    list<SizeT> geo_slot_indices;

    affine_body_dynamics->for_each(
        geo_slots,
        [&](const ForEachInfo& I, geometry::SimplicialComplex& sc)
        {
            auto geoI = I.global_index();
            auto uids = sc.meta().find<VectorXu64>(builtin::extra_constitution_uids);
            if(uids)
            {
                auto extra_uids = uids->view().front();
                for(auto extra_uid : extra_uids)
                {
                    if(extra_uid == uid)
                    {
                        geo_slot_indices.push_back(geoI);
                        break;
                    }
                }
            }
        });

    geo_infos.resize(geo_slot_indices.size());

    for(auto&& [i, geo_slot_index] : enumerate(geo_slot_indices))
    {
        geo_infos[i] = abd_geo_infos[geo_slot_index];
    }
}

Float AffineBodyExtraConstitution::BaseInfo::dt() const noexcept
{
    return m_dt;
}

muda::CBufferView<Vector12> AffineBodyExtraConstitution::BaseInfo::qs() const noexcept
{
    return m_impl->body_id_to_q.view();
}

muda::CBufferView<Vector12> AffineBodyExtraConstitution::BaseInfo::q_bars() const noexcept
{
    return m_impl->body_id_to_q_tilde.view();
}

span<const AffineBodyDynamics::GeoInfo> AffineBodyExtraConstitution::FilteredInfo::geo_infos() const noexcept
{
    return m_impl->geo_infos;
}

span<const Vector12> AffineBodyExtraConstitution::FilteredInfo::qs() noexcept
{
    return m_impl->abd().h_body_id_to_q;
}

span<const Vector12> AffineBodyExtraConstitution::FilteredInfo::q_vs() noexcept
{
    return m_impl->abd().h_body_id_to_q_v;
}
}  // namespace uipc::backend::cuda
