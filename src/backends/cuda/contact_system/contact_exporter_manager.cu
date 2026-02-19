#include <fmt/ranges.h>
#include <contact_system/contact_exporter_manager.h>
#include <contact_system/contact_system_feature.h>
#include <contact_system/contact_exporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ContactExporterManager);

void ContactExporterManager::do_build()
{
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
            logger::warn("Contact exporter for primitive type '{}'<{}> already exists, overwriting with <{}>.",
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
    m_contact_exporters.register_sim_system(*exporter);
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
        logger::warn(R"(Contact exporter for primitive type '{}' not found
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
