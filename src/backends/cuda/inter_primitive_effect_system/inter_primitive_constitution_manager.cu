#include <inter_primitive_effect_system/inter_primitive_constitution_manager.h>
#include <sim_engine.h>
#include <inter_primitive_effect_system/inter_primitive_constitution.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend
{
template <>
class backend::SimSystemCreator<cuda::InterPrimitiveConstitutionManager>
{
  public:
    static U<cuda::InterPrimitiveConstitutionManager> create(SimEngine& engine)
    {
        auto  scene = dynamic_cast<cuda::SimEngine&>(engine).world().scene();
        auto& types = scene.constitution_tabular().types();

        if(!types.contains(string{builtin::InterPrimitive}))
        {
            return nullptr;
        }

        return uipc::make_unique<cuda::InterPrimitiveConstitutionManager>(engine);
    }
};
}  // namespace uipc::backend

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(InterPrimitiveConstitutionManager);

void InterPrimitiveConstitutionManager::do_build(DyTopoEffectReporter::BuildInfo&)
{
    auto dt_attr                 = world().scene().config().find<Float>("dt");
    m_impl.dt                    = dt_attr->view()[0];
    m_impl.global_vertex_manager = require<GlobalVertexManager>();
}

void InterPrimitiveConstitutionManager::Impl::init(SceneVisitor& scene)
{
    auto constitution_view = constitutions.view();

    std::ranges::sort(constitution_view,
                      [](const InterPrimitiveConstitution* a, const InterPrimitiveConstitution* b)
                      { return a->uid() < b->uid(); });

    for(auto&& [i, c] : enumerate(constitution_view))
    {
        c->m_index             = i;  // assign index for each constitution
        uid_to_index[c->uid()] = i;  // build map
    }

    constitution_geo_info_offsets_counts.resize(constitution_view.size());
    auto geo_slots = scene.geometries();
    inter_geo_infos.reserve(geo_slots.size());

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
        inter_geo_infos.push_back(geo_info);
    }

    // stable sort by constitution uid to ensure that geometries from the same constitution are grouped together
    std::ranges::stable_sort(inter_geo_infos,
                             [](const InterGeoInfo& a, const InterGeoInfo& b)
                             { return a.constitution_uid < b.constitution_uid; });

    // compute offsets
    constitution_geo_info_offsets_counts.scan();

    // thus, the geo_infos are sorted by constitution uid and can use offset/count
    // to get a subspan for each constitution

    UIPC_ASSERT(constitution_geo_info_offsets_counts.total_count()
                    == inter_geo_infos.size(),
                "Mismatch in geometry count for constitutions: expected {}, found {}",
                constitution_geo_info_offsets_counts.total_count(),
                inter_geo_infos.size());


    // count geometries

    for(auto&& c : constitution_view)
    {
        FilteredInfo info{this, scene, c->m_index};
        c->init(info);
    }

    constitution_energy_offsets_counts.resize(constitution_view.size());
    constitution_gradient_offsets_counts.resize(constitution_view.size());
    constitution_hessian_offsets_counts.resize(constitution_view.size());
}

void InterPrimitiveConstitutionManager::Impl::compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info)
{
    auto constitution_view = constitutions.view();
    for(auto&& [i, c] : enumerate(constitution_view))
    {
        EnergyInfo this_info{this, c->m_index, dt, info.energies()};
        c->compute_energy(this_info);
    }
}

void InterPrimitiveConstitutionManager::Impl::compute_gradient_hessian(
    GlobalDyTopoEffectManager::GradientHessianInfo& info)
{
    auto constitution_view = constitutions.view();
    for(auto&& [i, c] : enumerate(constitution_view))
    {
        GradientHessianInfo this_info{
            this, c->m_index, info.gradient_only(), dt, info.gradients(), info.hessians()};
        c->compute_gradient_hessian(this_info);
    }
}

void InterPrimitiveConstitutionManager::do_init(DyTopoEffectReporter::InitInfo& info)
{
    auto scene = world().scene();
    m_impl.init(scene);
}

void InterPrimitiveConstitutionManager::add_constitution(InterPrimitiveConstitution* constitution) noexcept
{
    UIPC_ASSERT(constitution != nullptr, "Constitution must not be null");
    check_state(SimEngineState::BuildSystems, "add_constitution");
    m_impl.constitutions.register_sim_system(*constitution);
}

void InterPrimitiveConstitutionManager::do_report_gradient_hessian_extent(
    GlobalDyTopoEffectManager::GradientHessianExtentInfo& info)
{
    auto constitution_view = m_impl.constitutions.view();
    bool gradient_only     = info.gradient_only();
    // gradient and hessian counts
    {
        auto gradient_counts = m_impl.constitution_gradient_offsets_counts.counts();
        auto hessian_counts = m_impl.constitution_hessian_offsets_counts.counts();

        for(auto&& [i, c] : enumerate(constitution_view))
        {
            GradientHessianExtentInfo extent_info;
            extent_info.m_gradient_only = gradient_only;
            c->report_gradient_hessian_extent(extent_info);
            gradient_counts[i] = extent_info.m_gradient_count;
            UIPC_ASSERT(!(gradient_only && extent_info.m_hessian_count != 0),
                        "When gradient_only is true, hessian_count must be 0, but {} provides hessian count={}",
                        c->name(),
                        extent_info.m_hessian_count);
            hessian_counts[i] = gradient_only ? 0 : extent_info.m_hessian_count;
        }

        m_impl.constitution_gradient_offsets_counts.scan();
        m_impl.constitution_hessian_offsets_counts.scan();

        info.gradient_count(m_impl.constitution_gradient_offsets_counts.total_count());
        info.hessian_count(
            gradient_only ? 0 : m_impl.constitution_hessian_offsets_counts.total_count());
    }
}

void InterPrimitiveConstitutionManager::do_assemble(GlobalDyTopoEffectManager::GradientHessianInfo& info)
{
    m_impl.compute_gradient_hessian(info);
}

void InterPrimitiveConstitutionManager::do_compute_energy(GlobalDyTopoEffectManager::EnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void InterPrimitiveConstitutionManager::do_report_energy_extent(GlobalDyTopoEffectManager::EnergyExtentInfo& info)
{
    auto constitution_view = m_impl.constitutions.view();

    auto counts = m_impl.constitution_energy_offsets_counts.counts();

    for(auto&& [i, c] : enumerate(constitution_view))
    {
        EnergyExtentInfo extent_info;
        c->report_energy_extent(extent_info);
        counts[i] = extent_info.m_energy_count;
    }

    m_impl.constitution_energy_offsets_counts.scan();

    info.energy_count(m_impl.constitution_energy_offsets_counts.total_count());
}

muda::BufferView<Float> InterPrimitiveConstitutionManager::EnergyInfo::energies() const noexcept
{
    auto [offset, count] = m_impl->constitution_energy_offsets_counts[m_index];
    return m_energies.subview(offset, count);
}

muda::DoubletVectorView<Float, 3> InterPrimitiveConstitutionManager::GradientHessianInfo::gradients() const noexcept
{
    auto [offset, count] = m_impl->constitution_gradient_offsets_counts[m_index];
    return m_gradients.subview(offset, count);
}

muda::TripletMatrixView<Float, 3> InterPrimitiveConstitutionManager::GradientHessianInfo::hessians() const noexcept
{
    auto [offset, count] = m_impl->constitution_hessian_offsets_counts[m_index];
    return m_hessians.subview(offset, count);
}

bool InterPrimitiveConstitutionManager::GradientHessianInfo::gradient_only() const noexcept
{
    return m_gradient_only;
}

span<const InterPrimitiveConstitutionManager::InterGeoInfo> InterPrimitiveConstitutionManager::FilteredInfo::inter_geo_infos() const noexcept
{
    auto [offset, count] = m_impl->constitution_geo_info_offsets_counts[m_index];
    return span{m_impl->inter_geo_infos}.subspan(offset, count);
}

S<geometry::GeometrySlot> InterPrimitiveConstitutionManager::FilteredInfo::geo_slot(IndexT id) const
{
    return scene->find_geometry(id);
}

S<geometry::GeometrySlot> InterPrimitiveConstitutionManager::FilteredInfo::rest_geo_slot(IndexT id) const
{
    return scene->find_rest_geometry(id);
}

muda::CBufferView<Vector3> InterPrimitiveConstitutionManager::BaseInfo::positions() const noexcept
{
    return m_impl->global_vertex_manager->positions();
}

Float InterPrimitiveConstitutionManager::BaseInfo::dt() const noexcept
{
    return m_impl->dt;
}

void InterPrimitiveConstitutionManager::GradientHessianExtentInfo::hessian_count(SizeT count) noexcept
{
    m_hessian_count = count;
}

void InterPrimitiveConstitutionManager::GradientHessianExtentInfo::gradient_count(SizeT count) noexcept
{
    m_gradient_count = count;
}

void InterPrimitiveConstitutionManager::EnergyExtentInfo::energy_count(SizeT count) noexcept
{
    m_energy_count = count;
}
}  // namespace uipc::backend::cuda