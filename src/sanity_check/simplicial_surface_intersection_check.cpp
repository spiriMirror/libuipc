#include <uipc/core/contact_model.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <sanity_checker.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/spread_sheet_io.h>
#include <context.h>
#include <uipc/geometry/utils/bvh.h>
#include <uipc/geometry/utils/intersection.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/map.h>

namespace std
{
// Vector2i  set comparison
template <>
struct less<uipc::Vector2i>
{
    bool operator()(const uipc::Vector2i& lhs, const uipc::Vector2i& rhs) const
    {
        return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
    }
};
}  // namespace std

namespace uipc::sanity_check
{
class SimplicialSurfaceIntersectionCheck final : public SanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 1;
    using SanityChecker::SanityChecker;

    geometry::BVH bvh;

  protected:
    virtual void build(backend::SceneVisitor& scene) override
    {
        auto enable_contact = scene.info()["contact"]["enable"].get<bool>();
        if(!enable_contact)
        {
            throw SanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    geometry::SimplicialComplex extract_intersected_mesh(const geometry::SimplicialComplex& scene_surface,
                                                         span<const IndexT> vert_intersected,
                                                         span<const IndexT> edge_intersected,
                                                         span<const IndexT> tri_intersected)
    {
        geometry::SimplicialComplex i_mesh;

        // 1) Build up the intersected vertices, edges, and triangles
        vector<SizeT> intersected_verts;
        vector<SizeT> intersected_edges;
        vector<SizeT> intersected_tris;
        {
            intersected_verts.reserve(vert_intersected.size());
            for(auto i = 0; i < vert_intersected.size(); i++)
            {
                if(vert_intersected[i] == 1)
                    intersected_verts.push_back(i);
            }

            intersected_edges.reserve(edge_intersected.size());
            for(auto i = 0; i < edge_intersected.size(); i++)
            {
                if(edge_intersected[i] == 1)
                    intersected_edges.push_back(i);
            }

            intersected_tris.reserve(tri_intersected.size());
            for(auto i = 0; i < tri_intersected.size(); i++)
            {
                if(tri_intersected[i] == 1)
                    intersected_tris.push_back(i);
            }
        }


        // 2) Copy attributes
        {
            i_mesh.vertices().resize(intersected_verts.size());
            i_mesh.vertices().copy_from(scene_surface.vertices(),
                                        geometry::AttributeCopy::pull(intersected_verts));

            i_mesh.edges().resize(intersected_edges.size());
            i_mesh.edges().copy_from(scene_surface.edges(),
                                     geometry::AttributeCopy::pull(intersected_edges));

            i_mesh.triangles().resize(intersected_tris.size());
            i_mesh.triangles().copy_from(scene_surface.triangles(),
                                         geometry::AttributeCopy::pull(intersected_tris));
        }

        // 3) Remap vertex indices
        {
            vector<IndexT> vertex_remap(scene_surface.vertices().size(), -1);
            for(auto [i, v] : enumerate(intersected_verts))
                vertex_remap[v] = i;

            auto Map = [&]<IndexT N>(const Eigen::Vector<IndexT, N>& V) -> Eigen::Vector<IndexT, N>
            {
                auto ret = V;
                for(auto& v : ret)
                    v = vertex_remap[v];
                return ret;
            };

            auto edge_topo_view = view(i_mesh.edges().topo());
            std::ranges::transform(edge_topo_view, edge_topo_view.begin(), Map);

            auto tri_topo_view = view(i_mesh.triangles().topo());
            std::ranges::transform(tri_topo_view, tri_topo_view.begin(), Map);
        }

        return i_mesh;
    }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        auto context = find<Context>();

        const geometry::SimplicialComplex& scene_surface =
            context->scene_simplicial_surface();

        const ContactTabular& contact_tabular = context->contact_tabular();

        auto Vs = scene_surface.vertices().size() ? scene_surface.positions().view() :
                                                    span<const Vector3>{};
        auto Es = scene_surface.edges().size() ? scene_surface.edges().topo().view() :
                                                 span<const Vector2i>{};
        auto Fs = scene_surface.triangles().size() ?
                      scene_surface.triangles().topo().view() :
                      span<const Vector3i>{};

        if(Vs.size() == 0 || Es.size() == 0 || Fs.size() == 0)  // no need to check intersection
            return SanityCheckResult::Success;

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_instance_id =
            scene_surface.vertices().find<IndexT>("sanity_check/instance_id");
        UIPC_ASSERT(attr_v_instance_id,
                    "`sanity_check/instance_id` is not found in scene surface");
        auto VInstanceIds = attr_v_instance_id->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

        vector<geometry::BVH::AABB> tri_aabbs(Fs.size());
        for(auto [i, f] : enumerate(Fs))
        {
            tri_aabbs[i].extend(Vs[f[0]]).extend(Vs[f[1]]).extend(Vs[f[2]]);
        }

        vector<geometry::BVH::AABB> edge_aabbs(Es.size());
        for(auto [i, e] : enumerate(Es))
        {
            edge_aabbs[i].extend(Vs[e[0]]).extend(Vs[e[1]]);
        }

        bvh.build(tri_aabbs);

        vector<IndexT> vertex_intersected(Vs.size(), 0);
        vector<IndexT> edge_intersected(Es.size(), 0);
        vector<IndexT> tri_intersected(Fs.size(), 0);

        auto& contact_table = context->contact_tabular();
        auto  objs          = this->objects();

        bool has_intersection = false;

        // key: {geo_id_0, geo_id_1}, value: {obj_id_0, obj_id_1}
        map<Vector2i, Vector2i> intersected_geo_ids;

        bvh.query(
            edge_aabbs,
            [&](IndexT i, IndexT j)
            {
                Vector2i E = Es[i];
                Vector3i F = Fs[j];

                // 1) if there is a common point, don't consider it as an intersection
                {
                    Vector2i sorted_E = E;
                    Vector3i sorted_F = F;

                    std::sort(sorted_E.begin(), sorted_E.end());
                    std::sort(sorted_F.begin(), sorted_F.end());

                    vector<int> same_points;
                    same_points.reserve(3);
                    std::ranges::set_intersection(
                        sorted_E, sorted_F, std::back_inserter(same_points));
                    if(same_points.size() > 0)
                        return;
                }

                auto L = CIds[E[0]];
                auto R = CIds[F[0]];

                const core::ContactModel& model = contact_table.at(L, R);

                // 2) if the contact model is not enabled, don't consider it as an intersection
                if(!model.is_enabled())
                    return;


                Vector3 uvw;
                Vector2 uv;
                bool    is_coplanar;

                bool intersected = geometry::tri_edge_intersect(
                    Vs[F[0]], Vs[F[1]], Vs[F[2]], Vs[E[0]], Vs[E[1]], is_coplanar, uvw, uv);

                if(intersected)
                {
                    edge_intersected[i] = 1;
                    tri_intersected[j]  = 1;

                    vertex_intersected[E[0]] = 1;
                    vertex_intersected[E[1]] = 1;

                    vertex_intersected[F[0]] = 1;
                    vertex_intersected[F[1]] = 1;
                    vertex_intersected[F[2]] = 1;

                    has_intersection = true;

                    auto GeoIdL = VGeoIds[E[0]];
                    auto GeoIdR = VGeoIds[F[1]];

                    auto ObjIdL = VObjectIds[E[0]];
                    auto ObjIdR = VObjectIds[F[1]];

                    if(GeoIdL > GeoIdR)
                    {
                        std::swap(GeoIdL, GeoIdR);
                        std::swap(ObjIdL, ObjIdR);
                    }

                    intersected_geo_ids[{GeoIdL, GeoIdR}] = {ObjIdL, ObjIdR};
                }
            });

        if(has_intersection)
        {
            auto& buffer = msg.message();

            for(auto& [GeoIds, ObjIds] : intersected_geo_ids)
            {
                auto obj_0 = objects().find(ObjIds[0]);
                auto obj_1 = objects().find(ObjIds[1]);

                UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
                UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}) in Object[{}({})] intersects with Geometry({}) in "
                               "Object[{}({})]\n",
                               GeoIds[0],
                               obj_0->name(),
                               obj_0->id(),
                               GeoIds[1],
                               obj_1->name(),
                               obj_1->id());
            }

            auto intersected_mesh = extract_intersected_mesh(
                scene_surface, vertex_intersected, edge_intersected, tri_intersected);

            fmt::format_to(std::back_inserter(buffer),
                           "Intersected mesh has {} vertices, {} edges, and {} triangles.\n",
                           intersected_mesh.vertices().size(),
                           intersected_mesh.edges().size(),
                           intersected_mesh.triangles().size());

            std::string name = "intersected_mesh";

            if(scene.info()["sanity_check"]["mode"] == "normal")
            {
                auto output_path = this_output_path();
                namespace fs     = std::filesystem;
                fs::path path{output_path};
                path /= fmt::format("{}.obj", name);
                auto path_str = path.string();

                geometry::SimplicialComplexIO io;
                io.write(path_str, intersected_mesh);
                fmt::format_to(std::back_inserter(buffer),
                               "Intersected mesh is saved at {}.\n",
                               path_str);
            }

            fmt::format_to(std::back_inserter(buffer),
                           "Create mesh [{}<{}>] for post-processing.",
                           name,
                           intersected_mesh.type());

            msg.geometries()[name] =
                uipc::make_shared<geometry::SimplicialComplex>(std::move(intersected_mesh));

            return SanityCheckResult::Error;
        }

        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(SimplicialSurfaceIntersectionCheck);
}  // namespace uipc::sanity_check
