#include <finite_element/finite_element_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
void FiniteElementExtraConstitution::do_build(FiniteElementEnergyProducer::BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();

    auto uids     = world().scene().constitution_tabular().uids();
    auto this_uid = uid();
    if(!std::ranges::binary_search(uids, this_uid))
    {
        throw SimSystemException(fmt::format("Extra Constitution UID ({}) not found in the constitution tabular",
                                             this_uid));
    }

    BuildInfo this_info;
    do_build(this_info);

    m_impl.finite_element_method->add_constitution(this);
}

U64 FiniteElementExtraConstitution::uid() const noexcept
{
    return get_uid();
}

span<const FiniteElementMethod::GeoInfo> FiniteElementExtraConstitution::geo_infos() const noexcept
{
    return m_impl.geo_infos;
}

void FiniteElementExtraConstitution::init()
{
    m_impl.init(uid(), world());

    // let the subclass do the rest of the initialization
    FilteredInfo info{&m_impl};
    do_init(info);
}

void FiniteElementExtraConstitution::do_compute_energy(FiniteElementEnergyProducer::ComputeEnergyInfo& info)
{
    ComputeEnergyInfo this_info{&m_impl.fem(), info.dt(), info.energies()};
    do_compute_energy(this_info);
}

void FiniteElementExtraConstitution::do_compute_gradient_hessian(
    FiniteElementEnergyProducer::ComputeGradientHessianInfo& info)
{
    ComputeGradientHessianInfo this_info{
        &m_impl.fem(), info.dt(), info.gradients(), info.hessians()};
    do_compute_gradient_hessian(this_info);
}

void FiniteElementExtraConstitution::Impl::init(U64 uid, backend::WorldVisitor& world)
{
    using ForEachInfo = FiniteElementMethod::ForEachInfo;

    // 1) Find the geometry slots that have the extra constitution uids containing the given uid
    auto& fem_geo_infos = finite_element_method->m_impl.geo_infos;
    auto  geo_slots     = world.scene().geometries();

    list<SizeT> geo_slot_indices;

    finite_element_method->for_each(
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
                        // spdlog::info("Extra constitution {} found in geometry slot {}", uid, I);
                        break;
                    }
                }
            }
        });

    geo_infos.resize(geo_slot_indices.size());

    for(auto&& [i, geo_slot_index] : enumerate(geo_slot_indices))
    {
        geo_infos[i] = fem_geo_infos[geo_slot_index];
    }
}

Float FiniteElementExtraConstitution::BaseInfo::dt() const noexcept
{
    return m_dt;
}

muda::CBufferView<Vector3> FiniteElementExtraConstitution::BaseInfo::xs() const noexcept
{
    return m_impl->xs.view();
}

muda::CBufferView<Vector3> FiniteElementExtraConstitution::BaseInfo::x_bars() const noexcept
{
    return m_impl->x_bars.view();
}

muda::CBufferView<Float> FiniteElementExtraConstitution::BaseInfo::thicknesses() const noexcept
{
    return m_impl->thicknesses.view();
}

span<const FiniteElementMethod::GeoInfo> FiniteElementExtraConstitution::FilteredInfo::geo_infos() const noexcept
{
    return m_impl->geo_infos;
}

span<const Vector3> FiniteElementExtraConstitution::FilteredInfo::positions() noexcept
{
    return m_impl->fem().h_positions;
}

span<const Vector3> FiniteElementExtraConstitution::FilteredInfo::rest_positions() noexcept
{
    return m_impl->fem().h_rest_positions;
}

span<const Float> FiniteElementExtraConstitution::FilteredInfo::thicknesses() noexcept
{
    return m_impl->fem().h_thicknesses;
}
}  // namespace uipc::backend::cuda