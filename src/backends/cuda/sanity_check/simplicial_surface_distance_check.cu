#include <type_define.h>
#include <uipc/core/contact_tabular.h>
#include <uipc/core/subscene_tabular.h>
#include <sanity_check/backend_sanity_checker.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/io/simplicial_complex_io.h>
#include <collision_detection/aabb.h>
#include <collision_detection/info_stackless_bvh.h>
#include <utils/distance/distance_flagged.h>
#include <utils/simplex_contact_mask_utils.h>
#include <muda/buffer.h>
#include <muda/launch.h>

namespace uipc::backend::cuda
{
class SimplicialSurfaceDistanceCheck final : public BackendSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 3;
    using BackendSanityChecker::BackendSanityChecker;

    struct ViolationResult
    {
        vector<Vector2i> pp_pairs;
        vector<Vector2i> pe_pairs;
        vector<Vector2i> pt_pairs;
        vector<Vector2i> ee_pairs;
        IndexT           total_violations = 0;
    };

    struct Impl
    {
        ViolationResult check(span<const Vector3>  Vs,
                              span<const Vector2i> Es,
                              span<const Vector3i> Fs,
                              span<const IndexT>   CodimIndices,
                              span<const Float>    h_thickness_span,
                              span<const Float>    h_d_hat_span,
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
            SizeT num_codim = CodimIndices.size();

            DeviceBuffer<Vector3>  positions(num_verts);
            DeviceBuffer<Vector2i> edges(num_edges);
            DeviceBuffer<Vector3i> triangles(num_tris);
            DeviceBuffer<IndexT>   codim_indices(num_codim);
            DeviceBuffer<Float>    thickness(num_verts);
            DeviceBuffer<Float>    d_hat(num_verts);

            DeviceBuffer<IndexT> vert_bids(num_verts);
            DeviceBuffer<IndexT> vert_cids(num_verts);
            DeviceBuffer<IndexT> vert_scids(num_verts);
            DeviceBuffer<IndexT> self_collision(num_verts);

            positions.view().copy_from(Vs.data());
            if(num_edges > 0)
                edges.view().copy_from(Es.data());
            if(num_tris > 0)
                triangles.view().copy_from(Fs.data());
            if(num_codim > 0)
                codim_indices.view().copy_from(CodimIndices.data());
            thickness.view().copy_from(h_thickness_span.data());
            d_hat.view().copy_from(h_d_hat_span.data());
            vert_bids.view().copy_from(h_vert_bids.data());
            vert_cids.view().copy_from(h_vert_cids.data());
            vert_scids.view().copy_from(h_vert_scids.data());
            self_collision.view().copy_from(h_vert_self_collision.data());

            DeviceBuffer<AABB> point_aabbs(num_verts);
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_verts,
                       [positions = positions.cviewer().name("positions"),
                        thickness = thickness.cviewer().name("thickness"),
                        d_hat     = d_hat.cviewer().name("d_hat"),
                        point_aabbs = point_aabbs.viewer().name("point_aabbs")] __device__(int i) mutable
                       {
                           Float expansion = d_hat(i);
                           Float extend_f  = thickness(i) + expansion;
                           Eigen::Vector3f ext =
                               Eigen::Vector3f::Constant(static_cast<float>(extend_f));
                           Eigen::Vector3f pos = positions(i).cast<float>();

                           AABB box;
                           box.extend(pos - ext);
                           box.extend(pos + ext);
                           point_aabbs(i) = box;
                       });

            DeviceBuffer<AABB> codim_point_aabbs(num_codim);
            if(num_codim > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_codim,
                           [positions = positions.cviewer().name("positions"),
                            thickness = thickness.cviewer().name("thickness"),
                            d_hat     = d_hat.cviewer().name("d_hat"),
                            codim_indices = codim_indices.cviewer().name("codim_indices"),
                            codim_aabbs = codim_point_aabbs.viewer().name(
                                "codim_aabbs")] __device__(int i) mutable
                           {
                               IndexT p            = codim_indices(i);
                               Float  expansion    = d_hat(p);
                               Float  extend_f     = thickness(p) + expansion;
                               Eigen::Vector3f ext = Eigen::Vector3f::Constant(
                                   static_cast<float>(extend_f));
                               Eigen::Vector3f pos = positions(p).cast<float>();

                               AABB box;
                               box.extend(pos - ext);
                               box.extend(pos + ext);
                               codim_aabbs(i) = box;
                           });
            }

            DeviceBuffer<AABB> edge_aabbs(num_edges);
            if(num_edges > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_edges,
                           [positions = positions.cviewer().name("positions"),
                            edges     = edges.cviewer().name("edges"),
                            thickness = thickness.cviewer().name("thickness"),
                            d_hat     = d_hat.cviewer().name("d_hat"),
                            edge_aabbs = edge_aabbs.viewer().name("edge_aabbs")] __device__(int i) mutable
                           {
                               Vector2i E  = edges(i);
                               Float thick = thickness(E[0]) + thickness(E[1]);
                               Float expansion = (d_hat(E[0]) + d_hat(E[1])) / 2.0;
                               Float           extend_f = thick + expansion;
                               Eigen::Vector3f ext = Eigen::Vector3f::Constant(
                                   static_cast<float>(extend_f));
                               Eigen::Vector3f v0 = positions(E[0]).cast<float>();
                               Eigen::Vector3f v1 = positions(E[1]).cast<float>();

                               AABB box;
                               box.extend(v0 - ext);
                               box.extend(v0 + ext);
                               box.extend(v1 - ext);
                               box.extend(v1 + ext);
                               edge_aabbs(i) = box;
                           });
            }

            DeviceBuffer<AABB> tri_aabbs(num_tris);
            if(num_tris > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_tris,
                           [positions = positions.cviewer().name("positions"),
                            triangles = triangles.cviewer().name("triangles"),
                            thickness = thickness.cviewer().name("thickness"),
                            d_hat     = d_hat.cviewer().name("d_hat"),
                            tri_aabbs = tri_aabbs.viewer().name("tri_aabbs")] __device__(int i) mutable
                           {
                               Vector3i F  = triangles(i);
                               Float thick = thickness(F[0]) + thickness(F[1])
                                             + thickness(F[2]);
                               Float expansion =
                                   (d_hat(F[0]) + d_hat(F[1]) + d_hat(F[2])) / 3.0;
                               Float           extend_f = thick + expansion;
                               Eigen::Vector3f ext = Eigen::Vector3f::Constant(
                                   static_cast<float>(extend_f));
                               Eigen::Vector3f v0 = positions(F[0]).cast<float>();
                               Eigen::Vector3f v1 = positions(F[1]).cast<float>();
                               Eigen::Vector3f v2 = positions(F[2]).cast<float>();

                               AABB box;
                               box.extend(v0 - ext);
                               box.extend(v0 + ext);
                               box.extend(v1 - ext);
                               box.extend(v1 + ext);
                               box.extend(v2 - ext);
                               box.extend(v2 + ext);
                               tri_aabbs(i) = box;
                           });
            }

            auto& point_bids = vert_bids;
            auto& point_cids = vert_cids;

            DeviceBuffer<IndexT> codim_bids(num_codim);
            DeviceBuffer<IndexT> codim_cids(num_codim);
            if(num_codim > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_codim,
                           [codim_indices = codim_indices.cviewer().name("codim_indices"),
                            vert_bids = vert_bids.cviewer().name("vert_bids"),
                            vert_cids = vert_cids.cviewer().name("vert_cids"),
                            codim_bids = codim_bids.viewer().name("codim_bids"),
                            codim_cids = codim_cids.viewer().name("codim_cids")] __device__(int i) mutable
                           {
                               IndexT p      = codim_indices(i);
                               codim_bids(i) = vert_bids(p);
                               codim_cids(i) = vert_cids(p);
                           });
            }

            DeviceBuffer<IndexT> edge_bids(num_edges);
            DeviceBuffer<IndexT> edge_cids(num_edges);
            if(num_edges > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_edges,
                           [edges     = edges.cviewer().name("edges"),
                            vert_bids = vert_bids.cviewer().name("vert_bids"),
                            vert_cids = vert_cids.cviewer().name("vert_cids"),
                            edge_bids = edge_bids.viewer().name("edge_bids"),
                            edge_cids = edge_cids.viewer().name("edge_cids")] __device__(int i) mutable
                           {
                               edge_bids(i) = vert_bids(edges(i)[0]);
                               edge_cids(i) = vert_cids(edges(i)[0]);
                           });
            }

            DeviceBuffer<IndexT> tri_bids(num_tris);
            DeviceBuffer<IndexT> tri_cids(num_tris);
            if(num_tris > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_tris,
                           [triangles = triangles.cviewer().name("triangles"),
                            vert_bids = vert_bids.cviewer().name("vert_bids"),
                            vert_cids = vert_cids.cviewer().name("vert_cids"),
                            tri_bids  = tri_bids.viewer().name("tri_bids"),
                            tri_cids  = tri_cids.viewer().name("tri_cids")] __device__(int i) mutable
                           {
                               tri_bids(i) = vert_bids(triangles(i)[0]);
                               tri_cids(i) = vert_cids(triangles(i)[0]);
                           });
            }

            SizeT                  CN = contact_element_count;
            DeviceBuffer2D<IndexT> cmts;
            cmts.resize(Extent2D{CN, CN});
            cmts.view().copy_from(h_contact_mask.data());

            SizeT                  SN = subscene_element_count;
            DeviceBuffer2D<IndexT> scmts;
            scmts.resize(Extent2D{SN, SN});
            scmts.view().copy_from(h_subscene_mask.data());

            DeviceVar<IndexT> violation_count;
            BufferLaunch().fill(violation_count.view(), IndexT(0));

            auto cmts_v = cmts.viewer();

            InfoStacklessBVH::QueryBuffer pp_qbuf, pe_qbuf, pt_qbuf, ee_qbuf;

            auto vert_cids_v    = vert_cids.cviewer();
            auto vert_scids_v   = vert_scids.cviewer();
            auto self_coll_v    = self_collision.cviewer();
            auto contact_cmts_v = cmts.cviewer();
            auto subscene_cmts_v = scmts.cviewer();

            // Phase 1: CodimP vs AllP
            if(num_codim > 0 && num_verts > 0)
            {
                InfoStacklessBVH point_bvh;
                point_bvh.build(point_aabbs.view(), point_bids.view(), point_cids.view());

                auto violation_viewer = violation_count.viewer();
                auto pos_viewer       = positions.cviewer();
                auto thick_viewer     = thickness.cviewer();
                auto codim_viewer     = codim_indices.cviewer();

                point_bvh.query(
                    codim_point_aabbs.view(),
                    codim_bids.view(),
                    codim_cids.view(),
                    cmts.view(),
                    [cmts_v] __device__(const InfoStacklessBVH::NodePredInfo& info) -> bool
                    {
                        constexpr IndexT invalid = static_cast<IndexT>(-1);
                        bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                        && !cmts_v(info.query_cid, info.node_cid);
                        return !cid_cull;
                    },
                    [violation_viewer, pos_viewer, thick_viewer, codim_viewer,
                     vert_cids_v, vert_scids_v, self_coll_v,
                     contact_cmts_v, subscene_cmts_v] __device__(
                        const InfoStacklessBVH::LeafPredInfo& info) -> bool
                    {
                        IndexT codim_idx = info.i;
                        IndexT P         = info.j;
                        IndexT CodimP    = codim_viewer(codim_idx);

                        if(CodimP == P)
                            return false;

                        Vector2i scids = {vert_scids_v(CodimP), vert_scids_v(P)};
                        if(!allow_PP_contact(subscene_cmts_v, scids))
                            return false;

                        Vector2i cids = {vert_cids_v(CodimP), vert_cids_v(P)};
                        if(!allow_PP_contact(contact_cmts_v, cids))
                            return false;

                        if(info.bid_i == info.bid_j
                           && info.bid_i != static_cast<IndexT>(-1)
                           && !self_coll_v(CodimP))
                            return false;

                        Float D;
                        distance::point_point_distance2(pos_viewer(CodimP), pos_viewer(P), D);

                        Float thickness  = thick_viewer(CodimP) + thick_viewer(P);
                        Float thickness2 = thickness * thickness;

                        if(D <= thickness2)
                        {
                            atomicAdd(violation_viewer.data(), 1);
                            return true;
                        }
                        return false;
                    },
                    pp_qbuf);
            }

            // Phase 2: CodimP vs AllE
            if(num_codim > 0 && num_edges > 0)
            {
                InfoStacklessBVH edge_bvh;
                edge_bvh.build(edge_aabbs.view(), edge_bids.view(), edge_cids.view());

                auto violation_viewer = violation_count.viewer();
                auto pos_viewer       = positions.cviewer();
                auto thick_viewer     = thickness.cviewer();
                auto edge_viewer      = edges.cviewer();
                auto codim_viewer     = codim_indices.cviewer();

                edge_bvh.query(
                    codim_point_aabbs.view(),
                    codim_bids.view(),
                    codim_cids.view(),
                    cmts.view(),
                    [cmts_v] __device__(const InfoStacklessBVH::NodePredInfo& info) -> bool
                    {
                        constexpr IndexT invalid = static_cast<IndexT>(-1);
                        bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                        && !cmts_v(info.query_cid, info.node_cid);
                        return !cid_cull;
                    },
                    [violation_viewer, pos_viewer, thick_viewer, edge_viewer, codim_viewer,
                     vert_cids_v, vert_scids_v, self_coll_v,
                     contact_cmts_v, subscene_cmts_v] __device__(
                        const InfoStacklessBVH::LeafPredInfo& info) -> bool
                    {
                        IndexT   codim_idx = info.i;
                        IndexT   edge_idx  = info.j;
                        IndexT   CodimP    = codim_viewer(codim_idx);
                        Vector2i E         = edge_viewer(edge_idx);

                        if(CodimP == E[0] || CodimP == E[1])
                            return false;

                        Vector3i scids = {vert_scids_v(CodimP), vert_scids_v(E[0]), vert_scids_v(E[1])};
                        if(!allow_PE_contact(subscene_cmts_v, scids))
                            return false;

                        Vector3i cids = {vert_cids_v(CodimP), vert_cids_v(E[0]), vert_cids_v(E[1])};
                        if(!allow_PE_contact(contact_cmts_v, cids))
                            return false;

                        if(info.bid_i == info.bid_j
                           && info.bid_i != static_cast<IndexT>(-1)
                           && !self_coll_v(CodimP))
                            return false;

                        auto pe_flag = distance::point_edge_distance_flag(
                            pos_viewer(CodimP), pos_viewer(E[0]), pos_viewer(E[1]));
                        Float D;
                        distance::point_edge_distance2(pe_flag, pos_viewer(CodimP), pos_viewer(E[0]), pos_viewer(E[1]), D);

                        Float thickness = thick_viewer(CodimP) + thick_viewer(E[0]);
                        Float thickness2 = thickness * thickness;

                        if(D <= thickness2)
                        {
                            atomicAdd(violation_viewer.data(), 1);
                            return true;
                        }
                        return false;
                    },
                    pe_qbuf);
            }

            // Phase 3: AllP vs AllT
            if(num_verts > 0 && num_tris > 0)
            {
                InfoStacklessBVH tri_bvh;
                tri_bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());

                auto violation_viewer = violation_count.viewer();
                auto pos_viewer       = positions.cviewer();
                auto thick_viewer     = thickness.cviewer();
                auto tri_viewer       = triangles.cviewer();

                tri_bvh.query(
                    point_aabbs.view(),
                    point_bids.view(),
                    point_cids.view(),
                    cmts.view(),
                    [cmts_v] __device__(const InfoStacklessBVH::NodePredInfo& info) -> bool
                    {
                        constexpr IndexT invalid = static_cast<IndexT>(-1);
                        bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                        && !cmts_v(info.query_cid, info.node_cid);
                        return !cid_cull;
                    },
                    [violation_viewer, pos_viewer, thick_viewer, tri_viewer,
                     vert_cids_v, vert_scids_v, self_coll_v,
                     contact_cmts_v, subscene_cmts_v] __device__(
                        const InfoStacklessBVH::LeafPredInfo& info) -> bool
                    {
                        IndexT   P       = info.i;
                        IndexT   tri_idx = info.j;
                        Vector3i T       = tri_viewer(tri_idx);

                        if(P == T[0] || P == T[1] || P == T[2])
                            return false;

                        Vector4i scids = {vert_scids_v(P), vert_scids_v(T[0]), vert_scids_v(T[1]), vert_scids_v(T[2])};
                        if(!allow_PT_contact(subscene_cmts_v, scids))
                            return false;

                        Vector4i cids = {vert_cids_v(P), vert_cids_v(T[0]), vert_cids_v(T[1]), vert_cids_v(T[2])};
                        if(!allow_PT_contact(contact_cmts_v, cids))
                            return false;

                        if(info.bid_i == info.bid_j
                           && info.bid_i != static_cast<IndexT>(-1)
                           && !self_coll_v(P))
                            return false;

                        auto pt_flag = distance::point_triangle_distance_flag(
                            pos_viewer(P), pos_viewer(T[0]), pos_viewer(T[1]), pos_viewer(T[2]));
                        Float D;
                        distance::point_triangle_distance2(pt_flag,
                                                           pos_viewer(P),
                                                           pos_viewer(T[0]),
                                                           pos_viewer(T[1]),
                                                           pos_viewer(T[2]),
                                                           D);

                        Float thickness  = thick_viewer(P) + thick_viewer(T[0]);
                        Float thickness2 = thickness * thickness;

                        if(D <= thickness2)
                        {
                            atomicAdd(violation_viewer.data(), 1);
                            return true;
                        }
                        return false;
                    },
                    pt_qbuf);
            }

            // Phase 4: AllE vs AllE
            if(num_edges > 1)
            {
                InfoStacklessBVH edge_bvh;
                edge_bvh.build(edge_aabbs.view(), edge_bids.view(), edge_cids.view());

                auto violation_viewer = violation_count.viewer();
                auto pos_viewer       = positions.cviewer();
                auto thick_viewer     = thickness.cviewer();
                auto edge_viewer      = edges.cviewer();

                edge_bvh.detect(
                    cmts.view(),
                    [cmts_v] __device__(const InfoStacklessBVH::NodePredInfo& info) -> bool
                    {
                        constexpr IndexT invalid = static_cast<IndexT>(-1);
                        bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                        && !cmts_v(info.query_cid, info.node_cid);
                        return !cid_cull;
                    },
                    [violation_viewer, pos_viewer, thick_viewer, edge_viewer,
                     vert_cids_v, vert_scids_v, self_coll_v,
                     contact_cmts_v, subscene_cmts_v] __device__(
                        const InfoStacklessBVH::LeafPredInfo& info) -> bool
                    {
                        IndexT   ei = info.i;
                        IndexT   ej = info.j;
                        Vector2i E0 = edge_viewer(ei);
                        Vector2i E1 = edge_viewer(ej);

                        if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                            return false;

                        Vector4i scids = {vert_scids_v(E0[0]), vert_scids_v(E0[1]),
                                          vert_scids_v(E1[0]), vert_scids_v(E1[1])};
                        if(!allow_EE_contact(subscene_cmts_v, scids))
                            return false;

                        Vector4i cids = {vert_cids_v(E0[0]), vert_cids_v(E0[1]),
                                         vert_cids_v(E1[0]), vert_cids_v(E1[1])};
                        if(!allow_EE_contact(contact_cmts_v, cids))
                            return false;

                        if(info.bid_i == info.bid_j
                           && info.bid_i != static_cast<IndexT>(-1)
                           && !self_coll_v(E0[0]))
                            return false;

                        auto ee_flag = distance::edge_edge_distance_flag(
                            pos_viewer(E0[0]), pos_viewer(E0[1]),
                            pos_viewer(E1[0]), pos_viewer(E1[1]));
                        Float D;
                        distance::edge_edge_distance2(ee_flag,
                                                      pos_viewer(E0[0]),
                                                      pos_viewer(E0[1]),
                                                      pos_viewer(E1[0]),
                                                      pos_viewer(E1[1]),
                                                      D);

                        Float thickness = thick_viewer(E0[0]) + thick_viewer(E1[0]);
                        Float thickness2 = thickness * thickness;

                        if(D <= thickness2)
                        {
                            atomicAdd(violation_viewer.data(), 1);
                            return true;
                        }
                        return false;
                    },
                    ee_qbuf);
            }

            ViolationResult result;
            result.total_violations = violation_count;

            auto copy_pairs = [](InfoStacklessBVH::QueryBuffer& qbuf) -> vector<Vector2i>
            {
                vector<Vector2i> pairs(qbuf.size());
                if(!pairs.empty())
                    qbuf.view().copy_to(pairs.data());
                return pairs;
            };

            result.pp_pairs = copy_pairs(pp_qbuf);
            result.pe_pairs = copy_pairs(pe_qbuf);
            result.pt_pairs = copy_pairs(pt_qbuf);
            result.ee_pairs = copy_pairs(ee_qbuf);

            return result;
        }
    };

    static geometry::SimplicialComplex extract_close_mesh(const geometry::SimplicialComplex& scene_surface,
                                                          span<const IndexT> vertex_too_close,
                                                          span<const IndexT> edge_too_close,
                                                          span<const IndexT> tri_too_close)
    {
        geometry::SimplicialComplex mesh;

        vector<SizeT> close_verts;
        vector<SizeT> close_edges;
        vector<SizeT> close_tris;

        for(SizeT i = 0; i < vertex_too_close.size(); i++)
            if(vertex_too_close[i])
                close_verts.push_back(i);
        for(SizeT i = 0; i < edge_too_close.size(); i++)
            if(edge_too_close[i])
                close_edges.push_back(i);
        for(SizeT i = 0; i < tri_too_close.size(); i++)
            if(tri_too_close[i])
                close_tris.push_back(i);

        mesh.vertices().resize(close_verts.size());
        mesh.vertices().copy_from(scene_surface.vertices(),
                                  geometry::AttributeCopy::pull(close_verts));
        mesh.edges().resize(close_edges.size());
        mesh.edges().copy_from(scene_surface.edges(),
                               geometry::AttributeCopy::pull(close_edges));
        mesh.triangles().resize(close_tris.size());
        mesh.triangles().copy_from(scene_surface.triangles(),
                                   geometry::AttributeCopy::pull(close_tris));

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

        return mesh;
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

        if(Vs.size() == 0)
            return SanityCheckResult::Success;

        SizeT num_verts = Vs.size();

        vector<IndexT> h_codim_indices;
        {
            auto attr_dim = scene_surface.vertices().find<IndexT>("sanity_check/dim");
            UIPC_ASSERT(attr_dim, "`sanity_check/dim` is not found in scene surface, why can it happen?");
            auto dim = attr_dim->view();
            h_codim_indices.reserve(num_verts);
            for(SizeT i = 0; i < num_verts; ++i)
            {
                if(dim[i] <= 1)
                    h_codim_indices.push_back(static_cast<IndexT>(i));
            }
        }

        auto attr_v_thickness = scene_surface.vertices().find<Float>(builtin::thickness);
        auto VThickness =
            attr_v_thickness ? attr_v_thickness->view() : span<const Float>{};

        auto attr_v_d_hat = scene_surface.vertices().find<Float>("sanity_check/d_hat");
        UIPC_ASSERT(attr_v_d_hat, "`sanity_check/d_hat` is not found in scene surface");
        auto Vd_hats = attr_v_d_hat->view();

        vector<Float> h_thickness(num_verts, 0.0);
        vector<Float> h_d_hat(num_verts, 0.0);
        for(SizeT i = 0; i < num_verts; ++i)
        {
            h_thickness[i] = VThickness.empty() ? 0.0 : VThickness[i];
            h_d_hat[i]     = Vd_hats[i];
        }

        auto attr_v_instance_id =
            scene_surface.vertices().find<IndexT>("sanity_check/instance_id");
        UIPC_ASSERT(attr_v_instance_id,
                    "`sanity_check/instance_id` is not found in scene surface");
        auto VInstanceIds = attr_v_instance_id->view();

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        auto attr_scids = scene_surface.vertices().find<IndexT>(
            "sanity_check/subscene_contact_element_id");
        UIPC_ASSERT(attr_scids, "`sanity_check/subscene_contact_element_id` is not found in scene surface");
        auto SCIds = attr_scids->view();

        auto attr_self_collision =
            scene_surface.vertices().find<IndexT>("sanity_check/self_collision");
        UIPC_ASSERT(attr_self_collision,
                    "`sanity_check/self_collision` is not found in scene surface");
        auto SelfCollision = attr_self_collision->view();

        auto& contact_tabular = ctx.contact_tabular();
        SizeT CN              = contact_tabular.element_count();

        vector<IndexT> h_contact_mask(CN * CN);
        for(IndexT i = 0; i < (IndexT)CN; ++i)
            for(IndexT j = 0; j < (IndexT)CN; ++j)
                h_contact_mask[i * CN + j] = contact_tabular.at(i, j).is_enabled() ? 1 : 0;

        auto& subscene_tabular = ctx.subscene_tabular();
        SizeT SN               = subscene_tabular.element_count();

        vector<IndexT> h_subscene_mask(SN * SN);
        for(IndexT i = 0; i < (IndexT)SN; ++i)
            for(IndexT j = 0; j < (IndexT)SN; ++j)
                h_subscene_mask[i * SN + j] = subscene_tabular.at(i, j).is_enabled() ? 1 : 0;

        auto result =
            m_impl.check(Vs,
                         Es,
                         Fs,
                         span<const IndexT>{h_codim_indices},
                         span<const Float>{h_thickness},
                         span<const Float>{h_d_hat},
                         span<const IndexT>{VInstanceIds.data(), VInstanceIds.size()},
                         span<const IndexT>{CIds.data(), CIds.size()},
                         span<const IndexT>{SCIds.data(), SCIds.size()},
                         span<const IndexT>{SelfCollision.data(), SelfCollision.size()},
                         span<const IndexT>{h_contact_mask},
                         CN,
                         span<const IndexT>{h_subscene_mask},
                         SN);

        if(result.total_violations == 0)
            return SanityCheckResult::Success;

        SizeT num_edges = Es.size();
        SizeT num_tris  = Fs.size();

        vector<IndexT> vertex_too_close(num_verts, 0);
        vector<IndexT> edge_too_close(num_edges, 0);
        vector<IndexT> tri_too_close(num_tris, 0);

        auto mark_vert = [&](IndexT v) { vertex_too_close[v] = 1; };

        for(auto& p : result.pp_pairs)
        {
            IndexT codim_v = h_codim_indices[p[0]];
            mark_vert(codim_v);
            mark_vert(p[1]);
        }

        for(auto& p : result.pe_pairs)
        {
            IndexT codim_v = h_codim_indices[p[0]];
            mark_vert(codim_v);
            Vector2i E = Es[p[1]];
            mark_vert(E[0]);
            mark_vert(E[1]);
            edge_too_close[p[1]] = 1;
        }

        for(auto& p : result.pt_pairs)
        {
            mark_vert(p[0]);
            Vector3i F = Fs[p[1]];
            mark_vert(F[0]);
            mark_vert(F[1]);
            mark_vert(F[2]);
            tri_too_close[p[1]] = 1;
        }

        for(auto& p : result.ee_pairs)
        {
            Vector2i E0 = Es[p[0]];
            Vector2i E1 = Es[p[1]];
            mark_vert(E0[0]);
            mark_vert(E0[1]);
            mark_vert(E1[0]);
            mark_vert(E1[1]);
            edge_too_close[p[0]] = 1;
            edge_too_close[p[1]] = 1;
        }

        ::uipc::backend::SanityCheckMessageVisitor scmv{msg};
        auto& buffer = scmv.message();
        fmt::format_to(std::back_inserter(buffer),
                       "GPU distance check: detected {} primitive pair(s) closer than their combined thickness.\n",
                       result.total_violations);

        auto close_mesh =
            extract_close_mesh(scene_surface, vertex_too_close, edge_too_close, tri_too_close);

        fmt::format_to(std::back_inserter(buffer),
                       "Close mesh has {} vertices, {} edges, and {} triangles.\n",
                       close_mesh.vertices().size(),
                       close_mesh.edges().size(),
                       close_mesh.triangles().size());

        std::string name = "close_mesh";

        auto sanity_check_mode = ctx.config().find<std::string>("sanity_check/mode");
        if(sanity_check_mode->view()[0] == "normal")
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

        scmv.geometries()[name] =
            uipc::make_shared<geometry::SimplicialComplex>(std::move(close_mesh));

        return SanityCheckResult::Error;
    }

  private:
    Impl m_impl;
};

REGISTER_BACKEND_SANITY_CHECKER(SimplicialSurfaceDistanceCheck);
}  // namespace uipc::backend::cuda
