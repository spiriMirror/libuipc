#include <type_define.h>
#include <uipc/core/object.h>
#include <uipc/core/contact_tabular.h>
#include <uipc/core/subscene_tabular.h>
#include <sanity_check/backend_sanity_checker.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/io/simplicial_complex_io.h>
#include <uipc/common/map.h>
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

namespace uipc::backend::cuda
{
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

class SimplicialSurfaceIntersectionCheck final : public BackendSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 1;
    using BackendSanityChecker::BackendSanityChecker;

    struct Impl
    {
        vector<Vector2i> detect_intersections(span<const Vector3>  Vs,
                                              span<const Vector2i> Es,
                                              span<const Vector3i> Fs,
                                              span<const IndexT>   h_vert_bids,
                                              span<const IndexT>   h_vert_cids,
                                              span<const IndexT>   h_vert_scids,
                                              span<const IndexT>   h_vert_self_collision,
                                              span<const IndexT>   h_contact_mask,
                                              SizeT                contact_element_count,
                                              span<const IndexT>   h_subscene_mask,
                                              SizeT                subscene_element_count)
        {
            using namespace muda;

            SizeT num_verts = Vs.size();
            SizeT num_edges = Es.size();
            SizeT num_tris  = Fs.size();

            DeviceBuffer<Vector3>  positions(num_verts);
            DeviceBuffer<Vector2i> edges(num_edges);
            DeviceBuffer<Vector3i> triangles(num_tris);
            DeviceBuffer<IndexT>   vert_bids(num_verts);
            DeviceBuffer<IndexT>   vert_cids(num_verts);
            DeviceBuffer<IndexT>   vert_scids(num_verts);
            DeviceBuffer<IndexT>   self_collision(num_verts);

            positions.view().copy_from(Vs.data());
            edges.view().copy_from(Es.data());
            triangles.view().copy_from(Fs.data());
            vert_bids.view().copy_from(h_vert_bids.data());
            vert_cids.view().copy_from(h_vert_cids.data());
            vert_scids.view().copy_from(h_vert_scids.data());
            self_collision.view().copy_from(h_vert_self_collision.data());

            DeviceBuffer<AABB>   tri_aabbs(num_tris);
            DeviceBuffer<IndexT> tri_bids(num_tris);
            DeviceBuffer<IndexT> tri_cids(num_tris);
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_tris,
                       [positions = positions.cviewer().name("positions"),
                        triangles = triangles.cviewer().name("triangles"),
                        vert_bids = vert_bids.cviewer().name("vert_bids"),
                        vert_cids = vert_cids.cviewer().name("vert_cids"),
                        tri_aabbs = tri_aabbs.viewer().name("tri_aabbs"),
                        tri_bids  = tri_bids.viewer().name("tri_bids"),
                        tri_cids  = tri_cids.viewer().name("tri_cids")] __device__(int i) mutable
                       {
                           Vector3i F = triangles(i);
                           AABB     box;
                           box.extend(positions(F[0]).cast<float>());
                           box.extend(positions(F[1]).cast<float>());
                           box.extend(positions(F[2]).cast<float>());
                           tri_aabbs(i) = box;
                           tri_bids(i)  = vert_bids(F[0]);
                           tri_cids(i)  = vert_cids(F[0]);
                       });

            DeviceBuffer<AABB>   edge_aabbs(num_edges);
            DeviceBuffer<IndexT> edge_bids(num_edges);
            DeviceBuffer<IndexT> edge_cids(num_edges);
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_edges,
                       [positions  = positions.cviewer().name("positions"),
                        edges      = edges.cviewer().name("edges"),
                        vert_bids  = vert_bids.cviewer().name("vert_bids"),
                        vert_cids  = vert_cids.cviewer().name("vert_cids"),
                        edge_aabbs = edge_aabbs.viewer().name("edge_aabbs"),
                        edge_bids  = edge_bids.viewer().name("edge_bids"),
                        edge_cids  = edge_cids.viewer().name("edge_cids")] __device__(int i) mutable
                       {
                           Vector2i E = edges(i);
                           AABB     box;
                           box.extend(positions(E[0]).cast<float>());
                           box.extend(positions(E[1]).cast<float>());
                           edge_aabbs(i) = box;
                           edge_bids(i)  = vert_bids(E[0]);
                           edge_cids(i)  = vert_cids(E[0]);
                       });

            SizeT                  CN = contact_element_count;
            DeviceBuffer2D<IndexT> cmts;
            cmts.resize(Extent2D{CN, CN});
            cmts.view().copy_from(h_contact_mask.data());

            SizeT                  SN = subscene_element_count;
            DeviceBuffer2D<IndexT> scmts;
            scmts.resize(Extent2D{SN, SN});
            scmts.view().copy_from(h_subscene_mask.data());

            InfoStacklessBVH              bvh;
            InfoStacklessBVH::QueryBuffer qbuffer;

            bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());

            auto pos_viewer  = positions.cviewer();
            auto edge_viewer = edges.cviewer();
            auto tri_viewer  = triangles.cviewer();

            auto vert_cids_v    = vert_cids.cviewer();
            auto vert_scids_v   = vert_scids.cviewer();
            auto self_coll_v    = self_collision.cviewer();
            auto contact_cmts_v = cmts.cviewer();
            auto subscene_cmts_v = scmts.cviewer();

            auto cmts_v = cmts.viewer();

            bvh.query(
                edge_aabbs.view(),
                edge_bids.view(),
                edge_cids.view(),
                cmts.view(),
                [cmts_v] __device__(const InfoStacklessBVH::NodePredInfo& info) -> bool
                {
                    constexpr IndexT invalid = static_cast<IndexT>(-1);
                    bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                    && !cmts_v(info.query_cid, info.node_cid);
                    return !cid_cull;
                },
                [pos_viewer, edge_viewer, tri_viewer,
                 vert_cids_v, vert_scids_v, self_coll_v,
                 contact_cmts_v, subscene_cmts_v] __device__(
                    const InfoStacklessBVH::LeafPredInfo& info) -> bool
                {
                    Vector2i E = edge_viewer(info.i);
                    Vector3i F = tri_viewer(info.j);

                    if(E[0] == F[0] || E[0] == F[1] || E[0] == F[2] || E[1] == F[0]
                       || E[1] == F[1] || E[1] == F[2])
                        return false;

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

                    if(info.bid_i == info.bid_j
                       && info.bid_i != static_cast<IndexT>(-1)
                       && !self_coll_v(E[0]))
                        return false;

                    return tri_edge_intersect_device(pos_viewer(F[0]),
                                                     pos_viewer(F[1]),
                                                     pos_viewer(F[2]),
                                                     pos_viewer(E[0]),
                                                     pos_viewer(E[1]));
                },
                qbuffer);

            vector<Vector2i> h_pairs(qbuffer.size());
            if(!h_pairs.empty())
                qbuffer.view().copy_to(h_pairs.data());

            return h_pairs;
        }
    };

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
    virtual void build() override
    {
        auto enable_contact = context().config().find<IndexT>("contact/enable");
        if(!enable_contact->view()[0])
        {
            throw BackendSanityCheckerException("Contact is not enabled");
        }
    }

    virtual U64 get_id() const noexcept override { return SanityCheckerUID; }

    virtual SanityCheckResult do_check(core::SanityCheckMessage& msg) override
    {
        auto& ctx = context();

        const geometry::SimplicialComplex& scene_surface =
            ctx.scene_simplicial_surface();

        auto Vs = scene_surface.vertices().size() ? scene_surface.positions().view() :
                                                    span<const Vector3>{};
        auto Es = scene_surface.edges().size() ? scene_surface.edges().topo().view() :
                                                 span<const Vector2i>{};
        auto Fs = scene_surface.triangles().size() ?
                      scene_surface.triangles().topo().view() :
                      span<const Vector3i>{};

        if(Vs.size() == 0 || Es.size() == 0 || Fs.size() == 0)
            return SanityCheckResult::Success;

        auto& contact_tabular  = ctx.contact_tabular();
        auto& subscene_tabular = ctx.subscene_tabular();

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

        SizeT CN = contact_tabular.element_count();
        vector<IndexT> h_contact_mask(CN * CN);
        for(IndexT i = 0; i < (IndexT)CN; ++i)
            for(IndexT j = 0; j < (IndexT)CN; ++j)
                h_contact_mask[i * CN + j] = contact_tabular.at(i, j).is_enabled() ? 1 : 0;

        SizeT SN = subscene_tabular.element_count();
        vector<IndexT> h_subscene_mask(SN * SN);
        for(IndexT i = 0; i < (IndexT)SN; ++i)
            for(IndexT j = 0; j < (IndexT)SN; ++j)
                h_subscene_mask[i * SN + j] = subscene_tabular.at(i, j).is_enabled() ? 1 : 0;

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

        ::uipc::backend::SanityCheckMessageVisitor scmv{msg};
        auto& buffer = scmv.message();

        for(auto& [GeoIds, ObjIds] : intersected_geo_ids)
        {
            auto obj_0 = find_object(ObjIds[0]);
            auto obj_1 = find_object(ObjIds[1]);

            UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
            UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

            std::string name_0{obj_0->name()};
            std::string name_1{obj_1->name()};

            fmt::format_to(std::back_inserter(buffer),
                           "Geometry({}) in Object[{}({})] intersects with Geometry({}) in "
                           "Object[{}({})]\n",
                           GeoIds[0],
                           name_0,
                           obj_0->id(),
                           GeoIds[1],
                           name_1,
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

        auto sanity_check_mode = ctx.config().find<std::string>("sanity_check/mode");
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

        scmv.geometries()[name] =
            uipc::make_shared<geometry::SimplicialComplex>(std::move(intersected_mesh));

        return SanityCheckResult::Error;
    }

  private:
    Impl m_impl;
};

REGISTER_BACKEND_SANITY_CHECKER(SimplicialSurfaceIntersectionCheck);
}  // namespace uipc::backend::cuda
