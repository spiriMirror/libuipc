#include <finite_element/finite_element_extra_constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
void FiniteElementExtraConstitution::do_build()
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
    m_index = m_impl.fem().extra_constitution_uid_to_index[uid()];

    // let the subclass do the rest of the initialization
    FilteredInfo info{&m_impl};
    do_init(info);
}

void FiniteElementExtraConstitution::report_extent(ReportExtentInfo& info)
{
    do_report_extent(info);
    info.check(name());

    UIPC_ASSERT(!(info.gradient_only() && info.m_hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}.\n"
                "Ref: https://github.com/spiriMirror/libuipc/issues/295",
                name(),
                info.m_hessian_count);
}

void FiniteElementExtraConstitution::compute_energy(FiniteElementElastics::ComputeEnergyInfo& info)
{
    auto this_info = ComputeEnergyInfo{&m_impl, &info, info.dt()};
    do_compute_energy(this_info);
}

void FiniteElementExtraConstitution::compute_gradient_hessian(FiniteElementElastics::ComputeGradientHessianInfo& info)
{
    auto this_info = ComputeGradientHessianInfo{&m_impl, &info, info.dt()};
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
                        // logger::info("Extra constitution {} found in geometry slot {}", uid, I);
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
    return m_impl->finite_element_method->xs();
}

muda::CBufferView<Vector3> FiniteElementExtraConstitution::BaseInfo::x_bars() const noexcept
{
    return m_impl->finite_element_method->x_bars();
}

muda::CBufferView<Float> FiniteElementExtraConstitution::BaseInfo::thicknesses() const noexcept
{
    return m_impl->finite_element_method->thicknesses();
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