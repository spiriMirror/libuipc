#include <cuda_sanity_checker.h>
#include <context.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/common/map.h>
#include <primtive_contact.h>

#include <type_define.h>
#include <collision_detection/aabb.h>
#include <collision_detection/info_stackless_bvh.h>
#include <utils/simplex_contact_mask_utils.h>
#include <muda/buffer.h>
#include <muda/launch.h>

namespace std
{
template <>
struct less<uipc::Vector2i>
{
    bool operator()(const uipc::Vector2i& lhs, const uipc::Vector2i& rhs) const
    {
        return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
    }
};
}  // namespace std

namespace uipc::cuda_sanity_check
{
// Moeller-Trumbore ray-triangle intersection test (device function).
MUDA_GENERIC inline bool tri_edge_intersect_device(const Vector3& t0,
                                                   const Vector3& t1,
                                                   const Vector3& t2,
                                                   const Vector3& e0,
                                                   const Vector3& e1)
{
    constexpr Float eps = 1e-12;

    Vector3 dir = e1 - e0;
    Float   len = dir.norm();
    if(len < eps)
        return false;
    dir /= len;

    Vector3 edge1 = t1 - t0;
    Vector3 edge2 = t2 - t0;
    Vector3 h     = dir.cross(edge2);
    Float   a     = edge1.dot(h);

    if(a > -eps && a < eps)
        return false;

    Float   f = 1.0 / a;
    Vector3 s = e0 - t0;
    Float   u = f * s.dot(h);
    if(u < 0.0 || u > 1.0)
        return false;

    Vector3 q = s.cross(edge1);
    Float   v = f * dir.dot(q);
    if(v < 0.0 || u + v > 1.0)
        return false;

    Float t = f * edge2.dot(q);
    if(t < 0.0 || t > len)
        return false;

    return true;
}

class SimplicialSurfaceIntersectionCheck final : public CudaSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 1;
    using CudaSanityChecker::CudaSanityChecker;

    // GPU broadphase + narrowphase; returns intersecting (edge_idx, tri_idx) pairs on host.
    struct Impl
    {
        vector<Vector2i> detect_intersections(span<const Vector3>  Vs,
                                              span<const Vector2i> Es,
                                              span<const Vector3i> Fs,
                                              span<const IndexT>   vert_bids,
                                              span<const IndexT>   vert_cids,
                                              span<const IndexT>   vert_scids,
                                              span<const IndexT>   vert_self_collision,
                                              span<const IndexT>   h_contact_mask,
                                              SizeT                contact_element_count,
                                              span<const IndexT>   h_subscene_mask,
                                              SizeT                subscene_element_count)
        {
            using namespace uipc::backend::cuda;
            using namespace muda;

            SizeT num_verts = Vs.size();
            SizeT num_edges = Es.size();
            SizeT num_tris  = Fs.size();

            // 1) Copy scene data to GPU
            DeviceBuffer<Vector3>  d_positions(num_verts);
            DeviceBuffer<Vector2i> d_edges(num_edges);
            DeviceBuffer<Vector3i> d_triangles(num_tris);
            DeviceBuffer<IndexT>   d_vert_bids(num_verts);
            DeviceBuffer<IndexT>   d_vert_cids(num_verts);
            DeviceBuffer<IndexT>   d_vert_scids(num_verts);
            DeviceBuffer<IndexT>   d_self_collision(num_verts);

            d_positions.view().copy_from(Vs.data());
            d_edges.view().copy_from(Es.data());
            d_triangles.view().copy_from(Fs.data());
            d_vert_bids.view().copy_from(vert_bids.data());
            d_vert_cids.view().copy_from(vert_cids.data());
            d_vert_scids.view().copy_from(vert_scids.data());
            d_self_collision.view().copy_from(vert_self_collision.data());

            // 2) Build per-primitive BIDs/CIDs and AABBs on GPU
            DeviceBuffer<AABB>   d_tri_aabbs(num_tris);
            DeviceBuffer<IndexT> d_tri_bids(num_tris);
            DeviceBuffer<IndexT> d_tri_cids(num_tris);
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_tris,
                       [positions = d_positions.cviewer().name("positions"),
                        triangles = d_triangles.cviewer().name("triangles"),
                        v_bids    = d_vert_bids.cviewer().name("v_bids"),
                        v_cids    = d_vert_cids.cviewer().name("v_cids"),
                        tri_aabbs = d_tri_aabbs.viewer().name("tri_aabbs"),
                        tri_bids  = d_tri_bids.viewer().name("tri_bids"),
                        tri_cids = d_tri_cids.viewer().name("tri_cids")] __device__(int i) mutable
                       {
                           Vector3i F = triangles(i);
                           AABB     box;
                           box.extend(positions(F[0]).cast<float>());
                           box.extend(positions(F[1]).cast<float>());
                           box.extend(positions(F[2]).cast<float>());
                           tri_aabbs(i) = box;
                           tri_bids(i)  = v_bids(F[0]);
                           tri_cids(i)  = v_cids(F[0]);
                       });

            DeviceBuffer<AABB>   d_edge_aabbs(num_edges);
            DeviceBuffer<IndexT> d_edge_bids(num_edges);
            DeviceBuffer<IndexT> d_edge_cids(num_edges);
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_edges,
                       [positions  = d_positions.cviewer().name("positions"),
                        edges      = d_edges.cviewer().name("edges"),
                        v_bids     = d_vert_bids.cviewer().name("v_bids"),
                        v_cids     = d_vert_cids.cviewer().name("v_cids"),
                        edge_aabbs = d_edge_aabbs.viewer().name("edge_aabbs"),
                        edge_bids  = d_edge_bids.viewer().name("edge_bids"),
                        edge_cids = d_edge_cids.viewer().name("edge_cids")] __device__(int i) mutable
                       {
                           Vector2i E = edges(i);
                           AABB     box;
                           box.extend(positions(E[0]).cast<float>());
                           box.extend(positions(E[1]).cast<float>());
                           edge_aabbs(i) = box;
                           edge_bids(i)  = v_bids(E[0]);
                           edge_cids(i)  = v_cids(E[0]);
                       });

            // 3) Upload contact mask tabular to GPU
            SizeT                  CN = contact_element_count;
            DeviceBuffer2D<IndexT> d_cmts;
            d_cmts.resize(Extent2D{CN, CN});
            d_cmts.view().copy_from(h_contact_mask.data());

            // Upload subscene mask tabular to GPU
            SizeT                  SN = subscene_element_count;
            DeviceBuffer2D<IndexT> d_scmts;
            d_scmts.resize(Extent2D{SN, SN});
            d_scmts.view().copy_from(h_subscene_mask.data());

            // 4) BVH build + query
            InfoStacklessBVH              bvh;
            InfoStacklessBVH::QueryBuffer qbuffer;

            bvh.build(d_tri_aabbs.view(), d_tri_bids.view(), d_tri_cids.view());

            auto pos_viewer  = d_positions.cviewer();
            auto edge_viewer = d_edges.cviewer();
            auto tri_viewer  = d_triangles.cviewer();

            auto vert_cids_v    = d_vert_cids.cviewer();
            auto vert_scids_v   = d_vert_scids.cviewer();
            auto self_coll_v    = d_self_collision.cviewer();
            auto contact_cmts_v = d_cmts.cviewer();
            auto subscene_cmts_v = d_scmts.cviewer();

            auto cmts_v = d_cmts.viewer();
            auto node_pred = [cmts_v] __device__(const InfoStacklessBVH::NodePredInfo& info) -> bool
            {
                constexpr IndexT invalid = static_cast<IndexT>(-1);
                bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                && !cmts_v(info.query_cid, info.node_cid);
                return !cid_cull;
            };

            auto leaf_pred = [pos_viewer, edge_viewer, tri_viewer,
                              vert_cids_v, vert_scids_v, self_coll_v,
                              contact_cmts_v, subscene_cmts_v] __device__(
                                 const InfoStacklessBVH::LeafPredInfo& info) -> bool
            {
                Vector2i E = edge_viewer(info.i);
                Vector3i F = tri_viewer(info.j);

                // Skip if edge shares a vertex with triangle
                if(E[0] == F[0] || E[0] == F[1] || E[0] == F[2] || E[1] == F[0]
                   || E[1] == F[1] || E[1] == F[2])
                    return false;

                // Per-vertex subscene mask check: all cross-pairs between edge and triangle vertices
                {
                    bool ok = true;
                    for(int ei = 0; ei < 2 && ok; ++ei)
                        for(int fi = 0; fi < 3 && ok; ++fi)
                        {
                            IndexT scid_e = vert_scids_v(E[ei]);
                            IndexT scid_f = vert_scids_v(F[fi]);
                            if(!subscene_cmts_v(scid_e, scid_f))
                                ok = false;
                        }
                    if(!ok)
                        return false;
                }

                // Per-vertex contact mask check (all cross-pairs)
                {
                    bool ok = true;
                    for(int ei = 0; ei < 2 && ok; ++ei)
                        for(int fi = 0; fi < 3 && ok; ++fi)
                        {
                            IndexT cid_e = vert_cids_v(E[ei]);
                            IndexT cid_f = vert_cids_v(F[fi]);
                            if(!contact_cmts_v(cid_e, cid_f))
                                ok = false;
                        }
                    if(!ok)
                        return false;
                }

                // Self-collision check: same body + self_collision disabled => skip
                if(info.bid_i == info.bid_j
                   && info.bid_i != static_cast<IndexT>(-1)
                   && !self_coll_v(E[0]))
                    return false;

                return tri_edge_intersect_device(pos_viewer(F[0]),
                                                 pos_viewer(F[1]),
                                                 pos_viewer(F[2]),
                                                 pos_viewer(E[0]),
                                                 pos_viewer(E[1]));
            };

            bvh.query(d_edge_aabbs.view(),
                      d_edge_bids.view(),
                      d_edge_cids.view(),
                      d_cmts.view(),
                      node_pred,
                      leaf_pred,
                      qbuffer);

            // 5) Copy pairs back to host
            vector<Vector2i> h_pairs(qbuffer.size());
            if(!h_pairs.empty())
                qbuffer.view().copy_to(h_pairs.data());

            return h_pairs;
        }
    };

    // ---- CPU-side mesh extraction (same as CPU checker) ----
    static geometry::SimplicialComplex extract_intersected_mesh(
        const geometry::SimplicialComplex& scene_surface,
        span<const IndexT>                 vert_intersected,
        span<const IndexT>                 edge_intersected,
        span<const IndexT>                 tri_intersected)
    {
        geometry::SimplicialComplex i_mesh;

        vector<SizeT> intersected_verts;
        vector<SizeT> intersected_edges;
        vector<SizeT> intersected_tris;

        for(SizeT i = 0; i < vert_intersected.size(); i++)
            if(vert_intersected[i])
                intersected_verts.push_back(i);
        for(SizeT i = 0; i < edge_intersected.size(); i++)
            if(edge_intersected[i])
                intersected_edges.push_back(i);
        for(SizeT i = 0; i < tri_intersected.size(); i++)
            if(tri_intersected[i])
                intersected_tris.push_back(i);

        i_mesh.vertices().resize(intersected_verts.size());
        i_mesh.vertices().copy_from(scene_surface.vertices(),
                                    geometry::AttributeCopy::pull(intersected_verts));
        i_mesh.edges().resize(intersected_edges.size());
        i_mesh.edges().copy_from(scene_surface.edges(),
                                 geometry::AttributeCopy::pull(intersected_edges));
        i_mesh.triangles().resize(intersected_tris.size());
        i_mesh.triangles().copy_from(scene_surface.triangles(),
                                     geometry::AttributeCopy::pull(intersected_tris));

        // Remap vertex indices
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

        return i_mesh;
    }

  protected:
    virtual void build(backend::SceneVisitor& scene) override
    {
        auto enable_contact = scene.config().find<IndexT>("contact/enable");
        if(!enable_contact->view()[0])
        {
            throw CudaSanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    virtual SanityCheckResult do_check(backend::SceneVisitor& scene,
                                       backend::SanityCheckMessageVisitor& msg) noexcept override
    {
        auto context = find<Context>();

        const geometry::SimplicialComplex& scene_surface =
            context->scene_simplicial_surface();

        auto Vs = scene_surface.vertices().size() ? scene_surface.positions().view() :
                                                    span<const Vector3>{};
        auto Es = scene_surface.edges().size() ? scene_surface.edges().topo().view() :
                                                 span<const Vector2i>{};
        auto Fs = scene_surface.triangles().size() ?
                      scene_surface.triangles().topo().view() :
                      span<const Vector3i>{};

        if(Vs.size() == 0 || Es.size() == 0 || Fs.size() == 0)
            return SanityCheckResult::Success;

        auto& contact_tabular  = context->contact_tabular();
        auto& subscene_tabular = context->subscene_tabular();

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_scids = scene_surface.vertices().find<IndexT>(
            "sanity_check/subscene_contact_element_id");
        UIPC_ASSERT(attr_scids, "`sanity_check/subscene_contact_element_id` is not found in scene surface");
        auto SCIds = attr_scids->view();

        auto attr_v_instance_id =
            scene_surface.vertices().find<IndexT>("sanity_check/instance_id");
        UIPC_ASSERT(attr_v_instance_id,
                    "`sanity_check/instance_id` is not found in scene surface");
        auto VInstanceIds = attr_v_instance_id->view();

        auto attr_self_collision =
            scene_surface.vertices().find<IndexT>("sanity_check/self_collision");
        UIPC_ASSERT(attr_self_collision,
                    "`sanity_check/self_collision` is not found in scene surface");
        auto SelfCollision = attr_self_collision->view();

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

        // Build contact mask tabular on CPU for GPU BVH pruning
        SizeT CN = contact_tabular.element_count();

        vector<IndexT> h_contact_mask(CN * CN);
        for(IndexT i = 0; i < (IndexT)CN; ++i)
            for(IndexT j = 0; j < (IndexT)CN; ++j)
                h_contact_mask[i * CN + j] = contact_tabular.at(i, j).is_enabled() ? 1 : 0;

        // Build subscene mask tabular on CPU
        SizeT SN = subscene_tabular.element_count();

        vector<IndexT> h_subscene_mask(SN * SN);
        for(IndexT i = 0; i < (IndexT)SN; ++i)
            for(IndexT j = 0; j < (IndexT)SN; ++j)
                h_subscene_mask[i * SN + j] = subscene_tabular.at(i, j).is_enabled() ? 1 : 0;

        // ---- GPU broadphase + narrowphase (with full filtering on GPU) ----
        auto pairs = m_impl.detect_intersections(
            Vs,
            Es,
            Fs,
            span<const IndexT>{VInstanceIds.data(), VInstanceIds.size()},
            span<const IndexT>{CIds.data(), CIds.size()},
            span<const IndexT>{SCIds.data(), SCIds.size()},
            span<const IndexT>{SelfCollision.data(), SelfCollision.size()},
            span<const IndexT>{h_contact_mask},
            CN,
            span<const IndexT>{h_subscene_mask},
            SN);

        if(pairs.empty())
            return SanityCheckResult::Success;

        // ---- CPU: mark violated primitives from GPU pairs ----
        SizeT num_verts = Vs.size();
        SizeT num_edges = Es.size();
        SizeT num_tris  = Fs.size();

        vector<IndexT> vertex_intersected(num_verts, 0);
        vector<IndexT> edge_intersected(num_edges, 0);
        vector<IndexT> tri_intersected(num_tris, 0);

        map<Vector2i, Vector2i> intersected_geo_ids;

        for(auto& pair : pairs)
        {
            IndexT edge_idx = pair[0];
            IndexT tri_idx  = pair[1];

            Vector2i E = Es[edge_idx];
            Vector3i F = Fs[tri_idx];

            edge_intersected[edge_idx] = 1;
            tri_intersected[tri_idx]   = 1;

            vertex_intersected[E[0]] = 1;
            vertex_intersected[E[1]] = 1;
            vertex_intersected[F[0]] = 1;
            vertex_intersected[F[1]] = 1;
            vertex_intersected[F[2]] = 1;

            auto GeoIdL = VGeoIds[E[0]];
            auto GeoIdR = VGeoIds[F[0]];
            auto ObjIdL = VObjectIds[E[0]];
            auto ObjIdR = VObjectIds[F[0]];

            if(GeoIdL > GeoIdR)
            {
                std::swap(GeoIdL, GeoIdR);
                std::swap(ObjIdL, ObjIdR);
            }
            intersected_geo_ids[{GeoIdL, GeoIdR}] = {ObjIdL, ObjIdR};
        }

        // Report
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

        auto sanity_check_mode = scene.config().find<std::string>("sanity_check/mode");
        if(sanity_check_mode->view()[0] == "normal")
        {
            auto output_path = this_output_path();
            namespace fs     = std::filesystem;
            fs::path path{output_path};
            path /= fmt::format("{}.obj", name);
            auto path_str = path.string();

            geometry::SimplicialComplexIO io;
            {
                auto is_facet = intersected_mesh.edges().find<IndexT>(builtin::is_facet);
                if(!is_facet)
                    intersected_mesh.edges().create<IndexT>(builtin::is_facet, 0);
                auto is_facet_view = view(*is_facet);
                std::ranges::fill(is_facet_view, 1);
            }
            io.write(path_str, intersected_mesh);
            fmt::format_to(std::back_inserter(buffer), "Intersected mesh is saved at {}.\n", path_str);
        }

        fmt::format_to(std::back_inserter(buffer),
                       "Create mesh [{}<{}>] for post-processing.",
                       name,
                       intersected_mesh.type());

        msg.geometries()[name] =
            uipc::make_shared<geometry::SimplicialComplex>(std::move(intersected_mesh));

        return SanityCheckResult::Error;
    }

  private:
    Impl m_impl;
};

REGISTER_CUDA_SANITY_CHECKER(SimplicialSurfaceIntersectionCheck);
}  // namespace uipc::cuda_sanity_check
