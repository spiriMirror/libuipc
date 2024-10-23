#include <uipc/core/diff_sim.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/geometry_visitor.h>
#include <uipc/common/zip.h>

namespace uipc::core
{
diff_sim::ParameterCollection& DiffSim::parameters()
{
    return m_parameters;
}
const diff_sim::ParameterCollection& DiffSim::parameters() const
{
    return m_parameters;
}

diff_sim::SparseCOOView DiffSim::H() const
{
    return diff_sim::SparseCOOView({}, {}, {}, {});
}

diff_sim::SparseCOOView DiffSim::pGpP() const
{
    return diff_sim::SparseCOOView({}, {}, {}, {});
}

void DiffSim::init(backend::SceneVisitor& scene_visitor)
{
    auto geos      = scene_visitor.geometries();
    auto rest_geos = scene_visitor.rest_geometries();

    vector<std::string>                    collection_names;
    vector<geometry::AttributeCollection*> collections;

    // 1) Collect all [ diff_parm/xxx , xxx ] Attibute Collection Pairs
    for(auto&& [geo_slot, rest_geo_slot] : zip(geos, rest_geos))
    {
        std::array<geometry::Geometry*, 2> local_geos = {&geo_slot->geometry(),
                                                         &rest_geo_slot->geometry()};

        for(geometry::Geometry* geo : local_geos)
        {
            backend::GeometryVisitor geo_visitor{*geo};
            collection_names.clear();
            collections.clear();
            geo_visitor.collect_attribute_collections(collection_names, collections);

            for(auto&& [_, collection] : zip(collection_names, collections))
            {
                vector<std::string> attribute_names = collection->names();
                std::string_view    prefix          = "diff_parm/";

                auto [iter, end] = std::ranges::remove_if(
                    attribute_names,
                    [prefix](const std::string& name)
                    { return name.find(prefix) == std::string::npos; });

                attribute_names.erase(iter, end);

                for(const std::string& diff_parm_name : attribute_names)
                {
                    auto parm_name =
                        std::string_view{diff_parm_name}.substr(prefix.size());

                    auto diff_parm = collection->find(diff_parm_name);
                    auto parm      = collection->find(parm_name);

                    // connect diff_parm to parm
                    m_parameters.connect(diff_parm, parm);
                }
            }
        }
    }

    // 2) Build the connections
    m_parameters.build();

    // 3) Broadcast the parameters
    m_parameters.broadcast();
}
}  // namespace uipc::core
