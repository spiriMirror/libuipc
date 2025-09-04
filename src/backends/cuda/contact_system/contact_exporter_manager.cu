#include <fmt/ranges.h>
#include <contact_system/contact_exporter_manager.h>
#include <contact_system/contact_system_feature.h>
#include <contact_system/contact_exporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ContactExporterManager);

void ContactExporterManager::do_build()
{
    m_global_trajectory_filter     = require<GlobalTrajectoryFilter>();
    m_global_contact_manager       = require<GlobalContactManager>();
    m_contact_line_search_reporter = require<ContactLineSearchReporter>();

    auto overrider = std::make_shared<ContactSystemFeatureOverrider>(this);
    auto feature   = std::make_shared<core::ContactSystemFeature>(overrider);
    features().insert(feature);

    on_init_scene([&] { init(); });
}

void ContactExporterManager::init()
{
    auto exporters = m_contact_exporters.view();
    m_contact_prim_types.resize(exporters.size());

    for(auto&& [i, exporter] : enumerate(exporters))
    {
        auto prim_type = std::string{exporter->prim_type()};
        auto it        = m_exporter_map.find(prim_type);
        if(it != m_exporter_map.end())
        {
            spdlog::warn("Contact exporter for primitive type '{}'<{}> already exists, overwriting with <{}>.",
                         prim_type,
                         it->second->name(),
                         exporter->name());
            it->second = exporter;
        }
        else
        {
            m_exporter_map.emplace(prim_type, exporter);
        }

        m_contact_prim_types[i] = prim_type;
    }
}

void ContactExporterManager::compute_contact()
{
    if(m_global_trajectory_filter)
    {
        m_global_trajectory_filter->detect(0.0);
        m_global_trajectory_filter->filter_active();
    }
    m_global_contact_manager->compute_contact();
    m_contact_line_search_reporter->m_impl.compute_energy(false);
}

void ContactExporterManager::get_contact_energy(geometry::Geometry& energy_geo)
{
    auto& e = m_contact_line_search_reporter->m_impl.energy;
    energy_geo.instances().resize(1);  // we only have one energy instance (sum of all energies)
    auto energy = energy_geo.instances().find<Float>("energy");
    if(!energy)
    {
        energy = energy_geo.instances().create<Float>("energy");
    }
    auto energy_view = view(*energy);
    // copy from device to host
    energy_view[0] = e;
}

void ContactExporterManager::get_contact_gradient(geometry::Geometry& vert_grad)
{
    auto& g = m_global_contact_manager->m_impl.sorted_contact_gradient;

    vert_grad.instances().resize(g.doublet_count());
    auto i = vert_grad.instances().find<IndexT>("i");
    if(!i)
    {
        i = vert_grad.instances().create<IndexT>("i");
    }
    auto grad = vert_grad.instances().find<Vector3>("grad");
    if(!grad)
    {
        grad = vert_grad.instances().create<Vector3>("grad");
    }

    auto i_view = view(*i);
    g.indices().copy_to(i_view.data());

    auto grad_view = view(*grad);
    g.values().copy_to(grad_view.data());
}

void ContactExporterManager::get_contact_hessian(geometry::Geometry& vert_hess)
{
    auto& h = m_global_contact_manager->m_impl.sorted_contact_hessian;

    vert_hess.instances().resize(h.triplet_count());
    auto i = vert_hess.instances().find<IndexT>("i");
    if(!i)
    {
        i = vert_hess.instances().create<IndexT>("i");
    }
    auto j = vert_hess.instances().find<IndexT>("j");
    if(!j)
    {
        j = vert_hess.instances().create<IndexT>("j");
    }
    auto hess = vert_hess.instances().find<Matrix3x3>("hess");
    if(!hess)
    {
        hess = vert_hess.instances().create<Matrix3x3>("hess");
    }
    auto i_view    = view(*i);
    auto j_view    = view(*j);
    auto hess_view = view(*hess);

    h.row_indices().copy_to(i_view.data());
    h.col_indices().copy_to(j_view.data());
    h.values().copy_to(hess_view.data());
}

void ContactExporterManager::get_contact_energy(std::string_view    prim_type,
                                                geometry::Geometry& prim_energy)
{
    auto exporter = find_exporter(prim_type);
    if(!exporter)
        return;
    _create_prim_type_on_geo(prim_type, prim_energy);
    exporter->contact_energy(prim_type, prim_energy);
}

void ContactExporterManager::get_contact_gradient(std::string_view    prim_type,
                                                  geometry::Geometry& prim_grad)
{
    auto exporter = find_exporter(prim_type);
    if(!exporter)
        return;
    _create_prim_type_on_geo(prim_type, prim_grad);
    exporter->contact_gradient(prim_type, prim_grad);
}

void ContactExporterManager::get_contact_hessian(std::string_view    prim_type,
                                                 geometry::Geometry& prim_hess)
{
    auto exporter = find_exporter(prim_type);
    if(!exporter)
        return;
    _create_prim_type_on_geo(prim_type, prim_hess);
    exporter->contact_hessian(prim_type, prim_hess);
}

vector<std::string> ContactExporterManager::get_contact_primitive_types() const
{
    return m_contact_prim_types;
}

void ContactExporterManager::add_exporter(ContactExporter* exporter)
{
    UIPC_ASSERT(exporter, "Exporter must not be null");
    check_state(SimEngineState::BuildSystems, "add_exporter");
    m_contact_exporters.register_subsystem(*exporter);
}

ContactExporter* ContactExporterManager::find_exporter(std::string_view prim_type) const
{
    auto it = m_exporter_map.find(std::string{prim_type});
    if(it != m_exporter_map.end())
    {
        return it->second;
    }
    else
    {
        spdlog::warn(R"(Contact exporter for primitive type '{}' not found
Supported types are: [{}])",
                     prim_type,
                     fmt::join(m_contact_prim_types, ", "));

        return nullptr;
    }
}

void ContactExporterManager::_create_prim_type_on_geo(std::string_view prim_type_v,
                                                      geometry::Geometry& geo)
{
    auto prim_type = geo.meta().find<std::string>("prim_type");
    if(!prim_type)
    {
        prim_type = geo.meta().create<std::string>("prim_type");
    }
    view(*prim_type)[0] = prim_type_v;
}
}  // namespace uipc::backend::cuda
