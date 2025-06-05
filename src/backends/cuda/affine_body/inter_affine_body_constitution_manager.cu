#include <affine_body/inter_affine_body_constitution_manager.h>
#include <affine_body/inter_affine_body_constitution.h>
#include <sim_engine.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::InterAffineBodyConstitutionManager>
{
  public:
    static U<cuda::InterAffineBodyConstitutionManager> create(SimEngine& engine)
    {
        auto  scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto& types = scene.constitution_tabular().types();

        // inter affine body constitution manager requires
        // affine body and constraint types to be present
        if(types.find(string{builtin::AffineBody}) == types.end()
           || types.find(string{builtin::Constraint}) == types.end())
        {
            return nullptr;
        }

        return uipc::make_unique<cuda::InterAffineBodyConstitutionManager>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(InterAffineBodyConstitutionManager);

void InterAffineBodyConstitutionManager::do_build()
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
}

void InterAffineBodyConstitutionManager::Impl::init(SceneVisitor& scene)
{
    auto constitution_view = constitutions.view();

    std::ranges::sort(constitution_view,
                      [](const InterAffineBodyConstitution* a, const InterAffineBodyConstitution* b)
                      { return a->uid() < b->uid(); });

    for(auto&& [i, c] : enumerate(constitution_view))
    {
        c->m_index             = i;  // assign index for each constitution
        uid_to_index[c->uid()] = i;  // build map
    }

    constitution_geo_info_offsets_counts.resize(constitution_view.size());
    auto geo_slots = scene.geometries();
    geo_infos.reserve(geo_slots.size());

    span<IndexT> constitution_geo_info_counts =
        constitution_geo_info_offsets_counts.counts();

    for(auto&& [i, slot] : enumerate(geo_slots))
    {
        auto& geo = slot->geometry();
        auto  uid = geo.meta().find<U64>(builtin::constitution_uid);
        if(!uid)
            continue;

        U64 uid_value = uid->view()[0];
        if(!uid_to_index.contains(uid_value))
            continue;

        InterGeoInfo geo_info;
        geo_info.geo_slot_index   = i;
        geo_info.geo_id           = slot->id();
        geo_info.constitution_uid = uid_value;
        constitution_geo_info_counts[uid_to_index[uid_value]]++;
    }

    // stable sort by constitution uid to ensure that geometries from the same constitution are grouped together
    std::ranges::stable_sort(geo_infos,
                             [](const InterGeoInfo& a, const InterGeoInfo& b)
                             { return a.constitution_uid < b.constitution_uid; });

    // compute offsets
    constitution_geo_info_offsets_counts.scan();

    // thus, the geo_infos are sorted by constitution uid and can use offset/count
    // to get a subspan for each constitution

    UIPC_ASSERT(constitution_geo_info_offsets_counts.total_count() == geo_infos.size(),
                "Mismatch in geometry count for constitutions: expected {}, found {}",
                constitution_geo_info_offsets_counts.total_count(),
                geo_infos.size());


    // count geometries

    for(auto&& c : constitution_view)
    {
        FilteredInfo info{this, c->m_index};
        c->init(info);
    }
}

void InterAffineBodyConstitutionManager::init()
{
    auto scene = world().scene();
    m_impl.init(scene);
}

void InterAffineBodyConstitutionManager::add_constitution(InterAffineBodyConstitution* constitution) noexcept
{
    UIPC_ASSERT(constitution != nullptr, "Constitution must not be null");
    check_state(SimEngineState::BuildSystems, "add_constitution");
    m_impl.constitutions.register_subsystem(*constitution);
}

void InterAffineBodyConstitutionManager::GradientHessianExtentInfo::hessian_block_count(SizeT count) noexcept
{
    m_hessian_count = count;
}

void InterAffineBodyConstitutionManager::GradientHessianExtentInfo::gradient_segment_count(SizeT count) noexcept
{
    m_gradient_count = count;
}

void InterAffineBodyConstitutionManager::EnergyExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}

span<const InterAffineBodyConstitutionManager::InterGeoInfo> InterAffineBodyConstitutionManager::FilteredInfo::inter_geo_infos() const noexcept
{
    auto [offset, count] = m_impl->constitution_geo_info_offsets_counts[m_index];
    return span{m_impl->geo_infos}.subspan(offset, count);
}

span<const AffineBodyDynamics::GeoInfo> InterAffineBodyConstitutionManager::FilteredInfo::geo_infos() const noexcept
{
    return m_impl->affine_body_dynamics->m_impl.geo_infos;
}

IndexT InterAffineBodyConstitutionManager::FilteredInfo::geo_id_to_geo_info_index(IndexT geo_id) const noexcept
{
    return m_impl->affine_body_dynamics->m_impl.geo_id_to_geo_info_index[geo_id];
}
IndexT InterAffineBodyConstitutionManager::FilteredInfo::body_id(geometry::GeometrySlot& slot)
{
    auto geo_info_index = geo_id_to_geo_info_index(slot.id());
    UIPC_ASSERT(geo_info_index >= 0,
                "Geometry slot does not belong to any affine body constitution");
    const auto& geo_info = m_impl->affine_body_dynamics->m_impl.geo_infos[geo_info_index];
    UIPC_ASSERT(geo_info.body_count == 1,
                "Geometry slot must belong to exactly one affine body constitution");
    return m_impl->affine_body_dynamics->m_impl.geo_infos[geo_info_index].body_offset;
}
}  // namespace uipc::backend::cuda