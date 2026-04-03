#include <cuda_sanity_checker.h>
#include <context.h>
#include <primtive_contact.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/backend/visitors/scene_visitor.h>
#include <uipc/io/simplicial_complex_io.h>

#include <type_define.h>
#include <collision_detection/aabb.h>
#include <collision_detection/info_stackless_bvh.h>
#include <utils/distance/point_point.h>
#include <utils/distance/point_edge.h>
#include <utils/distance/point_triangle.h>
#include <utils/distance/edge_edge.h>
#include <utils/simplex_contact_mask_utils.h>
#include <muda/buffer.h>
#include <muda/launch.h>

namespace uipc::cuda_sanity_check
{
/**
 * @brief GPU-accelerated distance check for simplicial surfaces.
 *
 * Contact Pair Phases:
 * 1. CodimP-AllP: Codimensional points vs all points
 * 2. CodimP-AllE: Codimensional points vs all edges
 * 3. AllP-AllT:   All points vs all triangles
 * 4. AllE-AllE:   All edges vs all edges
 */
class SimplicialSurfaceDistanceCheck final : public CudaSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 3;
    using CudaSanityChecker::CudaSanityChecker;

    // Violation pairs from 4 BVH phases, returned to CPU for mesh export.
    struct ViolationResult
    {
        vector<Vector2i> pp_pairs;  // (codim_idx, vert_idx)
        vector<Vector2i> pe_pairs;  // (codim_idx, edge_idx)
        vector<Vector2i> pt_pairs;  // (vert_idx, tri_idx)
        vector<Vector2i> ee_pairs;  // (edge_idx_i, edge_idx_j)
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
            SizeT num_codim = CodimIndices.size();

            // -------------------------------------------------------------------
            // Copy data to GPU
            // -------------------------------------------------------------------
            DeviceBuffer<Vector3>  d_positions(num_verts);
            DeviceBuffer<Vector2i> d_edges(num_edges);
            DeviceBuffer<Vector3i> d_triangles(num_tris);
            DeviceBuffer<IndexT>   d_codim_indices(num_codim);
            DeviceBuffer<Float>    d_thickness(num_verts);
            DeviceBuffer<Float>    d_d_hat(num_verts);

            DeviceBuffer<IndexT> d_vert_bids(num_verts);
            DeviceBuffer<IndexT> d_vert_cids(num_verts);
            DeviceBuffer<IndexT> d_vert_scids(num_verts);
            DeviceBuffer<IndexT> d_self_collision(num_verts);

            d_positions.view().copy_from(Vs.data());
            if(num_edges > 0)
                d_edges.view().copy_from(Es.data());
            if(num_tris > 0)
                d_triangles.view().copy_from(Fs.data());
            if(num_codim > 0)
                d_codim_indices.view().copy_from(CodimIndices.data());
            d_thickness.view().copy_from(h_thickness_span.data());
            d_d_hat.view().copy_from(h_d_hat_span.data());
            d_vert_bids.view().copy_from(vert_bids.data());
            d_vert_cids.view().copy_from(vert_cids.data());
            d_vert_scids.view().copy_from(vert_scids.data());
            d_self_collision.view().copy_from(vert_self_collision.data());

            // -------------------------------------------------------------------
            // Build expanded AABBs on GPU
            // -------------------------------------------------------------------

            // Point AABBs (all vertices)
            DeviceBuffer<AABB> d_point_aabbs(num_verts);
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(num_verts,
                       [positions = d_positions.cviewer().name("positions"),
                        thickness = d_thickness.cviewer().name("thickness"),
                        d_hat     = d_d_hat.cviewer().name("d_hat"),
                        point_aabbs = d_point_aabbs.viewer().name("point_aabbs")] __device__(int i) mutable
                       {
                           Float expansion = d_hat(i);  // point_dcd_expansion
                           Float extend_f  = thickness(i) + expansion;
                           Eigen::Vector3f ext =
                               Eigen::Vector3f::Constant(static_cast<float>(extend_f));
                           Eigen::Vector3f pos = positions(i).cast<float>();

                           AABB box;
                           box.extend(pos - ext);
                           box.extend(pos + ext);
                           point_aabbs(i) = box;
                       });

            // Codimensional point AABBs
            DeviceBuffer<AABB> d_codim_point_aabbs(num_codim);
            if(num_codim > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_codim,
                           [positions = d_positions.cviewer().name("positions"),
                            thickness = d_thickness.cviewer().name("thickness"),
                            d_hat     = d_d_hat.cviewer().name("d_hat"),
                            codim_indices = d_codim_indices.cviewer().name("codim_indices"),
                            codim_aabbs = d_codim_point_aabbs.viewer().name(
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

            // Edge AABBs
            DeviceBuffer<AABB> d_edge_aabbs(num_edges);
            if(num_edges > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_edges,
                           [positions = d_positions.cviewer().name("positions"),
                            edges     = d_edges.cviewer().name("edges"),
                            thickness = d_thickness.cviewer().name("thickness"),
                            d_hat     = d_d_hat.cviewer().name("d_hat"),
                            edge_aabbs = d_edge_aabbs.viewer().name("edge_aabbs")] __device__(int i) mutable
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

            // Triangle AABBs
            DeviceBuffer<AABB> d_tri_aabbs(num_tris);
            if(num_tris > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_tris,
                           [positions = d_positions.cviewer().name("positions"),
                            triangles = d_triangles.cviewer().name("triangles"),
                            thickness = d_thickness.cviewer().name("thickness"),
                            d_hat     = d_d_hat.cviewer().name("d_hat"),
                            tri_aabbs = d_tri_aabbs.viewer().name("tri_aabbs")] __device__(int i) mutable
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

            // -------------------------------------------------------------------
            // Build per-primitive BIDs/CIDs on GPU from per-vertex data
            // -------------------------------------------------------------------
            // Point BIDs/CIDs = vertex BIDs/CIDs directly
            auto& d_point_bids = d_vert_bids;
            auto& d_point_cids = d_vert_cids;

            // Codim point BIDs/CIDs
            DeviceBuffer<IndexT> d_codim_bids(num_codim);
            DeviceBuffer<IndexT> d_codim_cids(num_codim);
            if(num_codim > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_codim,
                           [codim_indices = d_codim_indices.cviewer().name("codim_indices"),
                            v_bids = d_vert_bids.cviewer().name("v_bids"),
                            v_cids = d_vert_cids.cviewer().name("v_cids"),
                            codim_bids = d_codim_bids.viewer().name("codim_bids"),
                            codim_cids = d_codim_cids.viewer().name("codim_cids")] __device__(int i) mutable
                           {
                               IndexT p      = codim_indices(i);
                               codim_bids(i) = v_bids(p);
                               codim_cids(i) = v_cids(p);
                           });
            }

            // Edge BIDs/CIDs (use first vertex)
            DeviceBuffer<IndexT> d_edge_bids(num_edges);
            DeviceBuffer<IndexT> d_edge_cids(num_edges);
            if(num_edges > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_edges,
                           [edges     = d_edges.cviewer().name("edges"),
                            v_bids    = d_vert_bids.cviewer().name("v_bids"),
                            v_cids    = d_vert_cids.cviewer().name("v_cids"),
                            edge_bids = d_edge_bids.viewer().name("edge_bids"),
                            edge_cids = d_edge_cids.viewer().name("edge_cids")] __device__(int i) mutable
                           {
                               edge_bids(i) = v_bids(edges(i)[0]);
                               edge_cids(i) = v_cids(edges(i)[0]);
                           });
            }

            // Triangle BIDs/CIDs (use first vertex)
            DeviceBuffer<IndexT> d_tri_bids(num_tris);
            DeviceBuffer<IndexT> d_tri_cids(num_tris);
            if(num_tris > 0)
            {
                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(num_tris,
                           [triangles = d_triangles.cviewer().name("triangles"),
                            v_bids    = d_vert_bids.cviewer().name("v_bids"),
                            v_cids    = d_vert_cids.cviewer().name("v_cids"),
                            tri_bids  = d_tri_bids.viewer().name("tri_bids"),
                            tri_cids = d_tri_cids.viewer().name("tri_cids")] __device__(int i) mutable
                           {
                               tri_bids(i) = v_bids(triangles(i)[0]);
                               tri_cids(i) = v_cids(triangles(i)[0]);
                           });
            }

            // Upload contact mask tabular to GPU
            SizeT                  CN = contact_element_count;
            DeviceBuffer2D<IndexT> d_cmts;
            d_cmts.resize(Extent2D{CN, CN});
            d_cmts.view().copy_from(h_contact_mask.data());

            // Upload subscene mask tabular to GPU
            SizeT                  SN = subscene_element_count;
            DeviceBuffer2D<IndexT> d_scmts;
            d_scmts.resize(Extent2D{SN, SN});
            d_scmts.view().copy_from(h_subscene_mask.data());

            // Shared atomic violation counter
            DeviceVar<IndexT> d_violation_count;
            BufferLaunch().fill(d_violation_count.view(), IndexT(0));

            // Node predicate: use CID + contact mask for GPU-side subtree pruning
            auto cmts_v = d_cmts.viewer();
            auto node_pred = [cmts_v] __device__(
                                 const InfoStacklessBVH::NodePredInfo& info) -> bool
            {
                constexpr IndexT invalid = static_cast<IndexT>(-1);
                bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                                && !cmts_v(info.query_cid, info.node_cid);
                return !cid_cull;
            };

            // Per-phase query buffers (declared here so they survive the phase scopes)
            InfoStacklessBVH::QueryBuffer pp_qbuf, pe_qbuf, pt_qbuf, ee_qbuf;

            // Shared viewers for leaf predicates
            auto vert_cids_v    = d_vert_cids.cviewer();
            auto vert_scids_v   = d_vert_scids.cviewer();
            auto self_coll_v    = d_self_collision.cviewer();
            auto contact_cmts_v = d_cmts.cviewer();
            auto subscene_cmts_v = d_scmts.cviewer();

            // -------------------------------------------------------------------
            // Phase 1: CodimP vs AllP
            // -------------------------------------------------------------------
            if(num_codim > 0 && num_verts > 0)
            {
                InfoStacklessBVH point_bvh;

                point_bvh.build(d_point_aabbs.view(),
                                d_point_bids.view(),
                                d_point_cids.view());

                auto violation_viewer = d_violation_count.viewer();
                auto pos_viewer       = d_positions.cviewer();
                auto thick_viewer     = d_thickness.cviewer();
                auto codim_viewer     = d_codim_indices.cviewer();

                auto pp_leaf = [violation_viewer, pos_viewer, thick_viewer, codim_viewer,
                                vert_cids_v, vert_scids_v, self_coll_v,
                                contact_cmts_v, subscene_cmts_v] __device__(
                                   const InfoStacklessBVH::LeafPredInfo& info) -> bool
                {
                    IndexT codim_idx = info.i;  // index into codim_indices
                    IndexT P         = info.j;  // all-points index
                    IndexT CodimP    = codim_viewer(codim_idx);

                    // Skip self
                    if(CodimP == P)
                        return false;

                    // Per-vertex subscene mask check
                    Vector2i scids = {vert_scids_v(CodimP), vert_scids_v(P)};
                    if(!allow_PP_contact(subscene_cmts_v, scids))
                        return false;

                    // Per-vertex contact mask check
                    Vector2i cids = {vert_cids_v(CodimP), vert_cids_v(P)};
                    if(!allow_PP_contact(contact_cmts_v, cids))
                        return false;

                    // Self-collision check: same body + self_collision disabled => skip
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
                };

                point_bvh.query(d_codim_point_aabbs.view(),
                                d_codim_bids.view(),
                                d_codim_cids.view(),
                                d_cmts.view(),
                                node_pred,
                                pp_leaf,
                                pp_qbuf);
            }

            // -------------------------------------------------------------------
            // Phase 2: CodimP vs AllE
            // -------------------------------------------------------------------
            if(num_codim > 0 && num_edges > 0)
            {
                InfoStacklessBVH edge_bvh;

                edge_bvh.build(
                    d_edge_aabbs.view(), d_edge_bids.view(), d_edge_cids.view());

                auto violation_viewer = d_violation_count.viewer();
                auto pos_viewer       = d_positions.cviewer();
                auto thick_viewer     = d_thickness.cviewer();
                auto edge_viewer      = d_edges.cviewer();
                auto codim_viewer     = d_codim_indices.cviewer();

                auto pe_leaf = [violation_viewer, pos_viewer, thick_viewer, edge_viewer, codim_viewer,
                                vert_cids_v, vert_scids_v, self_coll_v,
                                contact_cmts_v, subscene_cmts_v] __device__(
                                   const InfoStacklessBVH::LeafPredInfo& info) -> bool
                {
                    IndexT   codim_idx = info.i;
                    IndexT   edge_idx  = info.j;
                    IndexT   CodimP    = codim_viewer(codim_idx);
                    Vector2i E         = edge_viewer(edge_idx);

                    // Skip if point is on the edge
                    if(CodimP == E[0] || CodimP == E[1])
                        return false;

                    // Per-vertex subscene mask check
                    Vector3i scids = {vert_scids_v(CodimP),
                                      vert_scids_v(E[0]),
                                      vert_scids_v(E[1])};
                    if(!allow_PE_contact(subscene_cmts_v, scids))
                        return false;

                    // Per-vertex contact mask check
                    Vector3i cids = {vert_cids_v(CodimP),
                                     vert_cids_v(E[0]),
                                     vert_cids_v(E[1])};
                    if(!allow_PE_contact(contact_cmts_v, cids))
                        return false;

                    // Self-collision check
                    if(info.bid_i == info.bid_j
                       && info.bid_i != static_cast<IndexT>(-1)
                       && !self_coll_v(CodimP))
                        return false;

                    Float D;
                    distance::point_edge_distance2(
                        pos_viewer(CodimP), pos_viewer(E[0]), pos_viewer(E[1]), D);

                    Float thickness = thick_viewer(CodimP) + thick_viewer(E[0]);
                    Float thickness2 = thickness * thickness;

                    if(D <= thickness2)
                    {
                        atomicAdd(violation_viewer.data(), 1);
                        return true;
                    }
                    return false;
                };

                edge_bvh.query(d_codim_point_aabbs.view(),
                               d_codim_bids.view(),
                               d_codim_cids.view(),
                               d_cmts.view(),
                               node_pred,
                               pe_leaf,
                               pe_qbuf);
            }

            // -------------------------------------------------------------------
            // Phase 3: AllP vs AllT
            // -------------------------------------------------------------------
            if(num_verts > 0 && num_tris > 0)
            {
                InfoStacklessBVH tri_bvh;

                tri_bvh.build(d_tri_aabbs.view(), d_tri_bids.view(), d_tri_cids.view());

                auto violation_viewer = d_violation_count.viewer();
                auto pos_viewer       = d_positions.cviewer();
                auto thick_viewer     = d_thickness.cviewer();
                auto tri_viewer       = d_triangles.cviewer();

                auto pt_leaf = [violation_viewer, pos_viewer, thick_viewer, tri_viewer,
                                vert_cids_v, vert_scids_v, self_coll_v,
                                contact_cmts_v, subscene_cmts_v] __device__(
                                   const InfoStacklessBVH::LeafPredInfo& info) -> bool
                {
                    IndexT   P       = info.i;
                    IndexT   tri_idx = info.j;
                    Vector3i T       = tri_viewer(tri_idx);

                    // Skip if point is on the triangle
                    if(P == T[0] || P == T[1] || P == T[2])
                        return false;

                    // Per-vertex subscene mask check
                    Vector4i scids = {vert_scids_v(P),
                                      vert_scids_v(T[0]),
                                      vert_scids_v(T[1]),
                                      vert_scids_v(T[2])};
                    if(!allow_PT_contact(subscene_cmts_v, scids))
                        return false;

                    // Per-vertex contact mask check
                    Vector4i cids = {vert_cids_v(P),
                                     vert_cids_v(T[0]),
                                     vert_cids_v(T[1]),
                                     vert_cids_v(T[2])};
                    if(!allow_PT_contact(contact_cmts_v, cids))
                        return false;

                    // Self-collision check
                    if(info.bid_i == info.bid_j
                       && info.bid_i != static_cast<IndexT>(-1)
                       && !self_coll_v(P))
                        return false;

                    Float D;
                    distance::point_triangle_distance2(pos_viewer(P),
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
                };

                tri_bvh.query(d_point_aabbs.view(),
                              d_point_bids.view(),
                              d_point_cids.view(),
                              d_cmts.view(),
                              node_pred,
                              pt_leaf,
                              pt_qbuf);
            }

            // -------------------------------------------------------------------
            // Phase 4: AllE vs AllE (self-collision within edge set)
            // -------------------------------------------------------------------
            if(num_edges > 1)
            {
                InfoStacklessBVH edge_bvh;

                edge_bvh.build(
                    d_edge_aabbs.view(), d_edge_bids.view(), d_edge_cids.view());

                auto violation_viewer = d_violation_count.viewer();
                auto pos_viewer       = d_positions.cviewer();
                auto thick_viewer     = d_thickness.cviewer();
                auto edge_viewer      = d_edges.cviewer();

                auto ee_leaf = [violation_viewer, pos_viewer, thick_viewer, edge_viewer,
                                vert_cids_v, vert_scids_v, self_coll_v,
                                contact_cmts_v, subscene_cmts_v] __device__(
                                   const InfoStacklessBVH::LeafPredInfo& info) -> bool
                {
                    IndexT   ei = info.i;
                    IndexT   ej = info.j;
                    Vector2i E0 = edge_viewer(ei);
                    Vector2i E1 = edge_viewer(ej);

                    // Skip if edges share a vertex
                    if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                        return false;

                    // Per-vertex subscene mask check
                    Vector4i scids = {vert_scids_v(E0[0]),
                                      vert_scids_v(E0[1]),
                                      vert_scids_v(E1[0]),
                                      vert_scids_v(E1[1])};
                    if(!allow_EE_contact(subscene_cmts_v, scids))
                        return false;

                    // Per-vertex contact mask check
                    Vector4i cids = {vert_cids_v(E0[0]),
                                     vert_cids_v(E0[1]),
                                     vert_cids_v(E1[0]),
                                     vert_cids_v(E1[1])};
                    if(!allow_EE_contact(contact_cmts_v, cids))
                        return false;

                    // Self-collision check
                    if(info.bid_i == info.bid_j
                       && info.bid_i != static_cast<IndexT>(-1)
                       && !self_coll_v(E0[0]))
                        return false;

                    Float D;
                    distance::edge_edge_distance2(pos_viewer(E0[0]),
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
                };

                // Use detect() for self-collision (edges vs edges in the same BVH)
                edge_bvh.detect(d_cmts.view(), node_pred, ee_leaf, ee_qbuf);
            }

            // -------------------------------------------------------------------
            // Collect results from all phases
            // -------------------------------------------------------------------
            ViolationResult result;
            result.total_violations = d_violation_count;

            // Copy pairs from each phase's qbuffer
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

        // Remap vertex indices
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

        if(Vs.size() == 0)
            return SanityCheckResult::Success;

        SizeT num_verts = Vs.size();

        // -------------------------------------------------------------------
        // Collect codimensional vertex indices on CPU (dim <= 1)
        // -------------------------------------------------------------------
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

        // Retrieve per-vertex thickness and d_hat
        auto attr_v_thickness = scene_surface.vertices().find<Float>(builtin::thickness);
        auto VThickness =
            attr_v_thickness ? attr_v_thickness->view() : span<const Float>{};

        auto attr_v_d_hat = scene_surface.vertices().find<Float>("sanity_check/d_hat");
        UIPC_ASSERT(attr_v_d_hat, "`sanity_check/d_hat` is not found in scene surface");
        auto Vd_hats = attr_v_d_hat->view();

        // Build host thickness + d_hat vectors for GPU upload
        vector<Float> h_thickness(num_verts, 0.0);
        vector<Float> h_d_hat(num_verts, 0.0);
        for(SizeT i = 0; i < num_verts; ++i)
        {
            h_thickness[i] = VThickness.empty() ? 0.0 : VThickness[i];
            h_d_hat[i]     = Vd_hats[i];
        }

        // Retrieve per-vertex instance_id (BID) and contact_element_id (CID)
        auto attr_v_instance_id =
            scene_surface.vertices().find<IndexT>("sanity_check/instance_id");
        UIPC_ASSERT(attr_v_instance_id,
                    "`sanity_check/instance_id` is not found in scene surface");
        auto VInstanceIds = attr_v_instance_id->view();

        auto attr_cids =
            scene_surface.vertices().find<IndexT>("sanity_check/contact_element_id");
        UIPC_ASSERT(attr_cids, "`sanity_check/contact_element_id` is not found in scene surface");
        auto CIds = attr_cids->view();

        // Retrieve per-vertex subscene_contact_element_id (SCID)
        auto attr_scids = scene_surface.vertices().find<IndexT>(
            "sanity_check/subscene_contact_element_id");
        UIPC_ASSERT(attr_scids, "`sanity_check/subscene_contact_element_id` is not found in scene surface");
        auto SCIds = attr_scids->view();

        // Retrieve per-vertex self_collision flag
        auto attr_self_collision =
            scene_surface.vertices().find<IndexT>("sanity_check/self_collision");
        UIPC_ASSERT(attr_self_collision,
                    "`sanity_check/self_collision` is not found in scene surface");
        auto SelfCollision = attr_self_collision->view();

        // Build contact mask tabular on CPU
        auto& contact_tabular = context->contact_tabular();
        SizeT CN              = contact_tabular.element_count();

        vector<IndexT> h_contact_mask(CN * CN);
        for(IndexT i = 0; i < (IndexT)CN; ++i)
            for(IndexT j = 0; j < (IndexT)CN; ++j)
                h_contact_mask[i * CN + j] = contact_tabular.at(i, j).is_enabled() ? 1 : 0;

        // Build subscene mask tabular on CPU
        auto& subscene_tabular = context->subscene_tabular();
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

        // ---- CPU: mark violated primitives from GPU pairs ----
        SizeT num_edges = Es.size();
        SizeT num_tris  = Fs.size();

        vector<IndexT> vertex_too_close(num_verts, 0);
        vector<IndexT> edge_too_close(num_edges, 0);
        vector<IndexT> tri_too_close(num_tris, 0);

        auto mark_vert = [&](IndexT v) { vertex_too_close[v] = 1; };

        // PP pairs: (codim_idx, vert_idx)
        for(auto& p : result.pp_pairs)
        {
            IndexT codim_v = h_codim_indices[p[0]];
            mark_vert(codim_v);
            mark_vert(p[1]);
        }

        // PE pairs: (codim_idx, edge_idx)
        for(auto& p : result.pe_pairs)
        {
            IndexT codim_v = h_codim_indices[p[0]];
            mark_vert(codim_v);
            Vector2i E = Es[p[1]];
            mark_vert(E[0]);
            mark_vert(E[1]);
            edge_too_close[p[1]] = 1;
        }

        // PT pairs: (vert_idx, tri_idx)
        for(auto& p : result.pt_pairs)
        {
            mark_vert(p[0]);
            Vector3i F = Fs[p[1]];
            mark_vert(F[0]);
            mark_vert(F[1]);
            mark_vert(F[2]);
            tri_too_close[p[1]] = 1;
        }

        // EE pairs: (edge_idx_i, edge_idx_j)
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

        // ---- Report ----
        auto& buffer = msg.message();
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

        auto sanity_check_mode = scene.config().find<std::string>("sanity_check/mode");
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

        msg.geometries()[name] =
            uipc::make_shared<geometry::SimplicialComplex>(std::move(close_mesh));

        return SanityCheckResult::Error;
    }

  private:
    Impl m_impl;
};

REGISTER_CUDA_SANITY_CHECKER(SimplicialSurfaceDistanceCheck);
}  // namespace uipc::cuda_sanity_check
