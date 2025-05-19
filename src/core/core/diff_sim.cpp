#include <uipc/core/diff_sim.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/backend/visitors/geometry_visitor.h>
#include <uipc/common/zip.h>
#include <uipc/backend/visitors/contact_tabular_visitor.h>
#include <uipc/core/internal/scene.h>

namespace uipc::geometry
{
template <>
class GeometryFriend<uipc::core::DiffSim>
{
  public:
    static auto& attribute_collections(Geometry& geometry) noexcept
    {
        return geometry.m_attribute_collections;
    }
};

}  // namespace uipc::geometry

namespace uipc::core
{


class DiffSim::Impl
{
  public:
    Impl() = default;

    void init(internal::Scene& scene)
    {
        auto geos      = scene.geometries().geometry_slots();
        auto rest_geos = scene.rest_geometries().geometry_slots();

        vector<std::string>                    collection_names;
        vector<geometry::AttributeCollection*> collections;

        // 1. Collect all [ diff/xxx , xxx ] Attibute Collection Pairs
        {
            // Common Function to detect diff/xxx and xxx
            auto detect_and_connect = [this](geometry::AttributeCollection& collection)
            {
                vector<std::string> attribute_names = collection.names();
                std::string_view    prefix          = "diff/";

                auto e = std::ranges::remove_if(
                    attribute_names,
                    [prefix](const std::string& name)
                    { return name.find(prefix) == std::string::npos; });

                attribute_names.erase(e.begin(), e.end());

                for(const std::string& diff_parm_name : attribute_names)
                {
                    auto parm_name =
                        std::string_view{diff_parm_name}.substr(prefix.size());

                    auto diff_parm = collection.find(diff_parm_name);
                    auto parm      = collection.find(parm_name);

                    UIPC_ASSERT(diff_parm, "Diff Parm `{}` not found, why can it happen?", diff_parm_name);
                    UIPC_ASSERT(parm,
                                "Diff Parm `{}` found, but Parm `{}` not, did you forget to create an attribute with name `{}` ?",
                                diff_parm_name,
                                parm_name,
                                parm_name);

                    // connect diff_parm to parm
                    parameters.connect(diff_parm, parm);
                }
            };

            // 1) Collect From Contact Tabular
            {
                auto& contact_tabular = scene.contact_tabular();
                backend::ContactTabularVisitor ctv{contact_tabular};
                detect_and_connect(ctv.contact_models());
            }

            // 2) Collect From Geometries
            using GF = geometry::GeometryFriend<DiffSim>;
            for(auto&& [geo_slot, rest_geo_slot] : zip(geos, rest_geos))
            {
                std::array<geometry::Geometry*, 2> local_geos = {
                    &geo_slot->geometry(), &rest_geo_slot->geometry()};

                for(geometry::Geometry* geo : local_geos)
                {
                    backend::GeometryVisitor geo_visitor{*geo};
                    collection_names.clear();
                    collections.clear();
                    geo_visitor.collect_attribute_collections(collection_names, collections);

                    auto& collection_map = GF::attribute_collections(*geo);

                    for(auto&& [name, collection] : collection_map)
                        detect_and_connect(*collection);
                }
            }
        }


        // 2. Build the connections
        parameters.build();

        // 3. Broadcast the parameters
        parameters.broadcast();
    }

    void clear()
    {
        H    = diff_sim::SparseCOOView{};
        pGpP = diff_sim::SparseCOOView{};

        need_backend_clear = true;
    }

    diff_sim::ParameterCollection parameters;
    diff_sim::SparseCOOView       H;
    diff_sim::SparseCOOView       pGpP;
    bool                          need_backend_clear = false;
};


DiffSim::DiffSim()
    : m_impl{uipc::make_unique<Impl>()}
{
}

DiffSim::~DiffSim() {}

diff_sim::ParameterCollection& DiffSim::parameters()
{
    return m_impl->parameters;
}

const diff_sim::ParameterCollection& DiffSim::parameters() const
{
    return m_impl->parameters;
}

void DiffSim::init(internal::Scene& scene)
{
    m_impl->init(scene);
}
}  // namespace uipc::core
