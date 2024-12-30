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
#include <uipc/geometry/utils/distance.h>
#include <uipc/geometry/utils/octree.h>
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
/**
 * @brief Check if simplicial surface distance is too close
 *
 * Contact Pair:
 *
 * CodimP-AllP: Codimensional Point v.s. All Point
 * CodimP-AllE: Codimensional Point v.s. All Edge
 * AllP-AllT: All Point v.s. All Triangle
 * AllE-AllE: All Edge v.s. All Edge
 *
 */
class SimplicialSurfaceDistanceCheck final : public SanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 3;
    using SanityChecker::SanityChecker;

    geometry::BVH tri_bvh;
    geometry::BVH edge_bvh;
    geometry::BVH point_bvh;

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

    vector<IndexT> CodimPs;

    void collect_codim_points(const geometry::SimplicialComplex& scene_surface)
    {
        auto attr_dim = scene_surface.vertices().find<IndexT>("sanity_check/dim");
        UIPC_ASSERT(attr_dim, "`sanity_check/dim` is not found in scene surface, why can it happen?");
        auto dim = attr_dim->view();
        CodimPs.reserve(scene_surface.vertices().size());
        for(auto [I, d] : enumerate(dim))
        {
            // codim 0D vert and vert from codim 1D edge
            if(d <= 1)
                CodimPs.push_back(I);
        }
    }

    geometry::SimplicialComplex extract_close_mesh(const geometry::SimplicialComplex& scene_surface,
                                                   span<const IndexT> vertex_too_close,
                                                   span<const IndexT> edge_too_close,
                                                   span<const IndexT> tri_too_close)
    {
        geometry::SimplicialComplex mesh;

        // 1) Build up the intersected vertices, edges, and triangles
        vector<SizeT> close_verts;
        vector<SizeT> close_edges;
        vector<SizeT> close_tris;
        {
            close_verts.reserve(vertex_too_close.size());
            for(auto i = 0; i < vertex_too_close.size(); i++)
            {
                if(vertex_too_close[i] == 1)
                    close_verts.push_back(i);
            }


            close_edges.reserve(edge_too_close.size());
            for(auto i = 0; i < edge_too_close.size(); i++)
            {
                if(edge_too_close[i] == 1)
                    close_edges.push_back(i);
            }

            close_tris.reserve(tri_too_close.size());
            for(auto i = 0; i < tri_too_close.size(); i++)
            {
                if(tri_too_close[i] == 1)
                    close_tris.push_back(i);
            }
        }


        // 2) Copy attributes
        {
            mesh.vertices().resize(close_verts.size());
            mesh.vertices().copy_from(scene_surface.vertices(),
                                      geometry::AttributeCopy::pull(close_verts));

            mesh.edges().resize(close_edges.size());
            mesh.edges().copy_from(scene_surface.edges(),
                                   geometry::AttributeCopy::pull(close_edges));

            mesh.triangles().resize(close_tris.size());
            mesh.triangles().copy_from(scene_surface.triangles(),
                                       geometry::AttributeCopy::pull(close_tris));
        }

        // 3) Remap vertex indices
        {
            vector<IndexT> vertex_remap(scene_surface.vertices().size(), -1);
            for(auto [i, v] : enumerate(close_verts))
                vertex_remap[v] = i;

            auto Map = [&]<IndexT N>(const Eigen::Vector<IndexT, N>& V) -> Eigen::Vector<IndexT, N>
            {
                auto ret = V;
                for(auto& v : ret)
                    v = vertex_remap[v];
                return ret;
            };

            auto edge_topo_view = view(mesh.edges().topo());
            std::ranges::transform(edge_topo_view, edge_topo_view.begin(), Map);

            auto tri_topo_view = view(mesh.triangles().topo());
            std::ranges::transform(tri_topo_view, tri_topo_view.begin(), Map);
        }

        return mesh;
    }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        auto context = find<Context>();

        auto d_hat = scene.info()["contact"]["d_hat"].get<Float>();

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

        if(Vs.size() == 0)  // no need to check distance
            return SanityCheckResult::Success;

        collect_codim_points(scene_surface);

        auto CodimPs = span{this->CodimPs};

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

        auto attr_v_thickness = scene_surface.vertices().find<Float>(builtin::thickness);
        auto VThickness =
            attr_v_thickness ? attr_v_thickness->view() : span<const Float>{};


        vector<geometry::BVH::AABB> codim_point_aabbs(CodimPs.size());
        for(auto [i, p] : enumerate(CodimPs))
        {
            auto thickness = VThickness.empty() ? 0 : VThickness[p];
            auto extend    = Vector3::Constant(thickness + d_hat);
            codim_point_aabbs[i].extend(Vs[p] - extend).extend(Vs[p] + extend);
        }

        vector<geometry::BVH::AABB> point_aabbs(Vs.size());
        for(auto [i, v] : enumerate(Vs))
        {
            auto thickness = VThickness.empty() ? 0 : VThickness[i];
            auto extend    = Vector3::Constant(thickness + d_hat);
            point_aabbs[i].extend(v - extend).extend(v + extend);
        }

        vector<geometry::BVH::AABB> edge_aabbs(Es.size());
        for(auto [i, e] : enumerate(Es))
        {
            auto thickness =
                VThickness.empty() ? 0 : VThickness[e[0]] + VThickness[e[1]];
            auto extend = Vector3::Constant(thickness + d_hat);
            edge_aabbs[i]
                .extend(Vs[e[0]] - extend)
                .extend(Vs[e[0]] + extend)
                .extend(Vs[e[1]] - extend)
                .extend(Vs[e[1]] + extend);
        }

        vector<geometry::BVH::AABB> tri_aabbs(Fs.size());
        for(auto [i, f] : enumerate(Fs))
        {
            auto thickness = VThickness.empty() ?
                                 0 :
                                 VThickness[f[0]] + VThickness[f[1]] + VThickness[f[2]];
            auto extend    = Vector3::Constant(thickness + d_hat);
            tri_aabbs[i]
                .extend(Vs[f[0]] - extend)
                .extend(Vs[f[0]] + extend)
                .extend(Vs[f[1]] - extend)
                .extend(Vs[f[1]] + extend)
                .extend(Vs[f[2]] - extend)
                .extend(Vs[f[2]] + extend);
        }

        tri_bvh.build(tri_aabbs);
        edge_bvh.build(edge_aabbs);
        point_bvh.build(point_aabbs);

        vector<IndexT> vertex_too_close(Vs.size(), 0);
        vector<IndexT> edge_too_close(Es.size(), 0);
        vector<IndexT> tri_too_close(Fs.size(), 0);

        auto& contact_table = context->contact_tabular();
        auto  objs          = this->objects();

        bool is_too_close = false;

        // key: {geo_id_0, geo_id_1}, value: {obj_id_0, obj_id_1}
        map<Vector2i, Vector2i> close_geo_ids;
        map<Vector2i, Vector2>  close_geo_distances;

        auto set_geo_distance = [&](const Vector2i& geo_ids, Float D, Float thickness2)
        {
            if(auto it = close_geo_distances.find(geo_ids);
               it == close_geo_distances.end())
            {
                close_geo_distances[geo_ids] = {D, thickness2};
            }
            else
            {
                if(D < it->second[0])
                {
                    it->second[0] = D;
                    it->second[1] = thickness2;
                }
            }
        };

        // 1) CodimP-AllP
        point_bvh.query(
            codim_point_aabbs,
            [&](IndexT i, IndexT j)
            {
                IndexT CodimP = CodimPs[i];
                IndexT P      = j;

                //1) if the two vertices are the same, don't consider it
                if(CodimP == P)
                    return;

                auto L = CIds[CodimP];
                auto R = CIds[P];

                const core::ContactModel& model = contact_table.at(L, R);

                // 2) if the contact model is not enabled, don't consider it
                if(!model.is_enabled())
                    return;

                Float D = geometry::point_point_squared_distance(Vs[CodimP], Vs[P]);

                Float thickness =
                    VThickness.empty() ? 0 : VThickness[CodimP] + VThickness[P];
                Float thickness2 = thickness * thickness;

                if(D <= thickness2)
                {
                    vertex_too_close[CodimP] = 1;
                    vertex_too_close[P]      = 1;

                    is_too_close = true;

                    Vector2i geo_ids{VGeoIds[CodimP], VGeoIds[P]};

                    close_geo_ids[geo_ids] = {VObjectIds[CodimP], VObjectIds[P]};

                    set_geo_distance(geo_ids, D, thickness2);
                }
            });

        // 2) CodimP-AllE
        edge_bvh.query(
            codim_point_aabbs,
            [&](IndexT i, IndexT j)
            {
                IndexT   CodimP = CodimPs[i];
                Vector2i E      = Es[j];

                // 1) if the point is on the edge, don't consider it
                if(CodimP == E[0] || CodimP == E[1])
                    return;

                auto L = CIds[CodimP];
                auto R = CIds[E[0]];

                const core::ContactModel& model = contact_table.at(L, R);

                // 2) if the contact model is not enabled, don't consider it
                if(!model.is_enabled())
                    return;

                Float D =
                    geometry::point_edge_squared_distance(Vs[CodimP], Vs[E[0]], Vs[E[1]]);

                Float thickness =
                    VThickness.empty() ? 0 : VThickness[CodimP] + VThickness[E[0]];
                Float thickness2 = thickness * thickness;
                if(D <= thickness2)
                {
                    vertex_too_close[CodimP] = 1;
                    edge_too_close[j]        = 1;

                    // also mark the vertex of the edge
                    vertex_too_close[E[0]] = 1;
                    vertex_too_close[E[1]] = 1;

                    is_too_close = true;

                    Vector2i geo_ids{VGeoIds[CodimP], VGeoIds[E[0]]};

                    close_geo_ids[geo_ids] = {VObjectIds[CodimP], VObjectIds[E[0]]};

                    set_geo_distance(geo_ids, D, thickness2);
                }
            });

        // 3) AllP-AllT
        tri_bvh.query(point_aabbs,
                      [&](IndexT i, IndexT j)
                      {
                          IndexT   P = i;
                          Vector3i T = Fs[j];

                          // 1) if the point is on the triangle, don't consider it
                          if(P == T[0] || P == T[1] || P == T[2])
                              return;

                          auto L = CIds[P];
                          auto R = CIds[T[0]];

                          const core::ContactModel& model = contact_table.at(L, R);

                          // 2) if the contact model is not enabled, don't consider it
                          if(!model.is_enabled())
                              return;

                          Float D = geometry::point_triangle_squared_distance(
                              Vs[P], Vs[T[0]], Vs[T[1]], Vs[T[2]]);

                          Float thickness = VThickness.empty() ?
                                                0 :
                                                VThickness[P] + VThickness[T[0]];

                          Float thickness2 = thickness * thickness;

                          if(D <= thickness2)
                          {
                              vertex_too_close[P] = 1;
                              tri_too_close[j]    = 1;

                              // also mark the vertices of the triangle
                              vertex_too_close[T[0]] = 1;
                              vertex_too_close[T[1]] = 1;
                              vertex_too_close[T[2]] = 1;

                              is_too_close = true;

                              Vector2i geo_ids{VGeoIds[P], VGeoIds[T[0]]};

                              close_geo_ids[geo_ids] = {VObjectIds[P], VObjectIds[T[0]]};

                              set_geo_distance(geo_ids, D, thickness2);
                          }
                      });

        // 4) AllE-AllE
        edge_bvh.query(
            edge_aabbs,
            [&](IndexT i, IndexT j)
            {
                Vector2i E0 = Es[i];
                Vector2i E1 = Es[j];

                // 1) if the two edges share a vertex, don't consider it
                if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                    return;

                auto L = CIds[E0[0]];
                auto R = CIds[E1[0]];

                const core::ContactModel& model = contact_table.at(L, R);

                // 2) if the contact model is not enabled, don't consider it
                if(!model.is_enabled())
                    return;

                Float D = geometry::edge_edge_squared_distance(
                    Vs[E0[0]], Vs[E0[1]], Vs[E1[0]], Vs[E1[1]]);

                Float thickness =
                    VThickness.empty() ? 0 : VThickness[E0[0]] + VThickness[E1[0]];

                Float thickness2 = thickness * thickness;

                if(D <= thickness2)
                {
                    edge_too_close[i] = 1;
                    edge_too_close[j] = 1;

                    // also mark the vertices of the edges
                    vertex_too_close[E0[0]] = 1;
                    vertex_too_close[E0[1]] = 1;
                    vertex_too_close[E1[0]] = 1;
                    vertex_too_close[E1[1]] = 1;

                    is_too_close = true;

                    Vector2i geo_ids{VGeoIds[E0[0]], VGeoIds[E1[0]]};

                    close_geo_ids[geo_ids] = {VObjectIds[E0[0]], VObjectIds[E1[0]]};

                    set_geo_distance(geo_ids, D, thickness2);
                }
            });

        if(is_too_close)
        {
            auto& buffer = msg.message();

            for(auto& [GeoIds, ObjIds] : close_geo_ids)
            {
                auto obj_0 = objects().find(ObjIds[0]);
                auto obj_1 = objects().find(ObjIds[1]);

                UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
                UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

                fmt::format_to(std::back_inserter(buffer),
                               "Geometry({}) in Object[{}({})] is too close (distance={}, thickness={}) to Geometry({}) in "
                               "Object[{}({})]\n",
                               GeoIds[0],
                               obj_0->name(),
                               obj_0->id(),
                               std::sqrt(close_geo_distances[GeoIds][0]),
                               std::sqrt(close_geo_distances[GeoIds][1]),
                               GeoIds[1],
                               obj_1->name(),
                               obj_1->id());
            }

            auto close_mesh = extract_close_mesh(
                scene_surface, vertex_too_close, edge_too_close, tri_too_close);

            fmt::format_to(std::back_inserter(buffer),
                           "Close mesh has {} vertices, {} edges, and {} triangles.\n",
                           close_mesh.vertices().size(),
                           close_mesh.edges().size(),
                           close_mesh.triangles().size());

            std::string name = "close_mesh";

            if(scene.info()["sanity_check"]["mode"] == "normal")
            {
                auto output_path = this_output_path();
                namespace fs     = std::filesystem;
                fs::path path{output_path};
                path /= fmt::format("{}.obj", name);
                auto path_str = path.string();

                geometry::SimplicialComplexIO io;
                io.write(path_str, close_mesh);
                fmt::format_to(std::back_inserter(buffer), "Close mesh is saved at {}.\n", path_str);
            }

            fmt::format_to(std::back_inserter(buffer),
                           "Create mesh [{}<{}>] for post-processing.",
                           name,
                           close_mesh.type());

            msg.geometries()[name] =
                uipc::make_shared<geometry::SimplicialComplex>(std::move(close_mesh));

            return SanityCheckResult::Error;
        }

        return SanityCheckResult::Success;
    };
};

REGISTER_SANITY_CHECKER(SimplicialSurfaceDistanceCheck);
}  // namespace uipc::sanity_check
