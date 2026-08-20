#include <type_define.h>
#include <uipc/core/contact_tabular.h>
#include <uipc/core/object.h>
#include <uipc/core/subscene_tabular.h>
#include <sanity_check/backend_sanity_checker.h>
#include <uipc/backend/visitors/sanity_check_message_visitor.h>
#include <uipc/geometry/simplicial_complex.h>
#include <uipc/builtin/attribute_name.h>
#include <uipc/common/map.h>
#include <uipc/io/simplicial_complex_io.h>
#include <collision_detection/aabb.h>
#include <collision_detection/info_stackless_bvh.h>
#include <utils/distance/distance_flagged.h>
#include <utils/simplex_contact_mask_utils.h>
#include <cuda_tool/cuda_tool.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void SimplicialSurfaceDistanceCheck_build_point_aabbs_kernel(
        cuda_tool::CBufferView<Vector3> positions,
        cuda_tool::CBufferView<Float>   thickness,
        cuda_tool::CBufferView<Float>   d_hat,
        cuda_tool::BufferView<AABB>     point_aabbs,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Float           expansion = d_hat(i);
        Float           extend_f  = thickness(i) + expansion;
        Eigen::Vector3f ext =
            Eigen::Vector3f::Constant(static_cast<float>(extend_f));
        Eigen::Vector3f pos = positions(i).cast<float>();

        AABB box;
        box.extend(pos - ext);
        box.extend(pos + ext);
        point_aabbs(i) = box;
    }

    __global__ void SimplicialSurfaceDistanceCheck_build_codim_point_aabbs_kernel(
        cuda_tool::CBufferView<Vector3> positions,
        cuda_tool::CBufferView<Float>   thickness,
        cuda_tool::CBufferView<Float>   d_hat,
        cuda_tool::CBufferView<IndexT>  codim_indices,
        cuda_tool::BufferView<AABB>     codim_aabbs,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
    }

    __global__ void SimplicialSurfaceDistanceCheck_build_edge_aabbs_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> edges,
        cuda_tool::CBufferView<Float>    thickness,
        cuda_tool::CBufferView<Float>    d_hat,
        cuda_tool::BufferView<AABB>      edge_aabbs,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
    }

    __global__ void SimplicialSurfaceDistanceCheck_build_tri_aabbs_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3i> triangles,
        cuda_tool::CBufferView<Float>    thickness,
        cuda_tool::CBufferView<Float>    d_hat,
        cuda_tool::BufferView<AABB>      tri_aabbs,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
    }

    __global__ void SimplicialSurfaceDistanceCheck_gather_codim_bids_cids_kernel(
        cuda_tool::CBufferView<IndexT> codim_indices,
        cuda_tool::CBufferView<IndexT> vert_bids,
        cuda_tool::CBufferView<IndexT> vert_cids,
        cuda_tool::BufferView<IndexT>  codim_bids,
        cuda_tool::BufferView<IndexT>  codim_cids,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        IndexT p      = codim_indices(i);
        codim_bids(i) = vert_bids(p);
        codim_cids(i) = vert_cids(p);
    }

    __global__ void SimplicialSurfaceDistanceCheck_gather_edge_bids_cids_kernel(
        cuda_tool::CBufferView<Vector2i> edges,
        cuda_tool::CBufferView<IndexT>   vert_bids,
        cuda_tool::CBufferView<IndexT>   vert_cids,
        cuda_tool::BufferView<IndexT>    edge_bids,
        cuda_tool::BufferView<IndexT>    edge_cids,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        edge_bids(i) = vert_bids(edges(i)[0]);
        edge_cids(i) = vert_cids(edges(i)[0]);
    }

    __global__ void SimplicialSurfaceDistanceCheck_gather_tri_bids_cids_kernel(
        cuda_tool::CBufferView<Vector3i> triangles,
        cuda_tool::CBufferView<IndexT>   vert_bids,
        cuda_tool::CBufferView<IndexT>   vert_cids,
        cuda_tool::BufferView<IndexT>    tri_bids,
        cuda_tool::BufferView<IndexT>    tri_cids,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        tri_bids(i) = vert_bids(triangles(i)[0]);
        tri_cids(i) = vert_cids(triangles(i)[0]);
    }

    struct SimplicialSurfaceDistanceCheck_NodePred
    {
        cuda_tool::Dense2D<IndexT> ContactMask;

        __device__ bool operator()(const InfoStacklessBVH::NodePredInfo& info) const
        {
            constexpr IndexT invalid = static_cast<IndexT>(-1);
            bool cid_cull = info.node_cid != invalid && info.query_cid != invalid
                            && !ContactMask(info.query_cid, info.node_cid);
            return !cid_cull;
        }
    };

    struct SimplicialSurfaceDistanceCheck_PP_LeafPred
    {
        cuda_tool::CBufferView<Vector3> Vs;
        cuda_tool::CBufferView<Float>   VThickness;
        cuda_tool::CBufferView<IndexT>  CodimPs;
        cuda_tool::CBufferView<IndexT>  CIds;
        cuda_tool::CBufferView<IndexT>  SCIds;
        cuda_tool::CBufferView<IndexT>  SelfCollision;
        cuda_tool::CDense2D<IndexT>     ContactTable;
        cuda_tool::CDense2D<IndexT>     SubsceneTable;

        __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
        {
            IndexT codim_idx = info.i;
            IndexT P         = info.j;
            IndexT CodimP    = CodimPs(codim_idx);

            if(CodimP == P)
                return false;

            Vector2i scids = {SCIds(CodimP), SCIds(P)};
            if(!allow_PP_contact(SubsceneTable, scids))
                return false;

            Vector2i cids = {CIds(CodimP), CIds(P)};
            if(!allow_PP_contact(ContactTable, cids))
                return false;

            if(info.bid_i == info.bid_j
               && info.bid_i != static_cast<IndexT>(-1)
               && !SelfCollision(CodimP))
                return false;

            Float D;
            distance::point_point_distance2(Vs(CodimP), Vs(P), D);

            Float thickness  = VThickness(CodimP) + VThickness(P);
            Float thickness2 = thickness * thickness;

            if(D <= thickness2)
                return true;
            return false;
        }
    };

    struct SimplicialSurfaceDistanceCheck_PE_LeafPred
    {
        cuda_tool::CBufferView<Vector3>  Vs;
        cuda_tool::CBufferView<Float>    VThickness;
        cuda_tool::CBufferView<Vector2i> Es;
        cuda_tool::CBufferView<IndexT>   CodimPs;
        cuda_tool::CBufferView<IndexT>   CIds;
        cuda_tool::CBufferView<IndexT>   SCIds;
        cuda_tool::CBufferView<IndexT>   SelfCollision;
        cuda_tool::CDense2D<IndexT>      ContactTable;
        cuda_tool::CDense2D<IndexT>      SubsceneTable;

        __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
        {
            IndexT   codim_idx = info.i;
            IndexT   edge_idx  = info.j;
            IndexT   CodimP    = CodimPs(codim_idx);
            Vector2i E         = Es(edge_idx);

            if(CodimP == E[0] || CodimP == E[1])
                return false;

            Vector3i scids = {SCIds(CodimP), SCIds(E[0]), SCIds(E[1])};
            if(!allow_PE_contact(SubsceneTable, scids))
                return false;

            Vector3i cids = {CIds(CodimP), CIds(E[0]), CIds(E[1])};
            if(!allow_PE_contact(ContactTable, cids))
                return false;

            if(info.bid_i == info.bid_j
               && info.bid_i != static_cast<IndexT>(-1)
               && !SelfCollision(CodimP))
                return false;

            auto pe_flag = distance::point_edge_distance_flag(
                Vs(CodimP), Vs(E[0]), Vs(E[1]));
            Float D;
            distance::point_edge_distance2(
                pe_flag, Vs(CodimP), Vs(E[0]), Vs(E[1]), D);

            Float thickness = VThickness(CodimP) + VThickness(E[0]);
            Float thickness2 = thickness * thickness;

            if(D <= thickness2)
                return true;
            return false;
        }
    };

    struct SimplicialSurfaceDistanceCheck_PT_LeafPred
    {
        cuda_tool::CBufferView<Vector3>  Vs;
        cuda_tool::CBufferView<Float>    VThickness;
        cuda_tool::CBufferView<Vector3i> Fs;
        cuda_tool::CBufferView<IndexT>   CIds;
        cuda_tool::CBufferView<IndexT>   SCIds;
        cuda_tool::CBufferView<IndexT>   SelfCollision;
        cuda_tool::CDense2D<IndexT>      ContactTable;
        cuda_tool::CDense2D<IndexT>      SubsceneTable;

        __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
        {
            IndexT   P       = info.i;
            IndexT   tri_idx = info.j;
            Vector3i T       = Fs(tri_idx);

            if(P == T[0] || P == T[1] || P == T[2])
                return false;

            Vector4i scids = {SCIds(P), SCIds(T[0]), SCIds(T[1]), SCIds(T[2])};
            if(!allow_PT_contact(SubsceneTable, scids))
                return false;

            Vector4i cids = {CIds(P), CIds(T[0]), CIds(T[1]), CIds(T[2])};
            if(!allow_PT_contact(ContactTable, cids))
                return false;

            if(info.bid_i == info.bid_j
               && info.bid_i != static_cast<IndexT>(-1) && !SelfCollision(P))
                return false;

            auto pt_flag = distance::point_triangle_distance_flag(
                Vs(P), Vs(T[0]), Vs(T[1]), Vs(T[2]));
            Float D;
            distance::point_triangle_distance2(
                pt_flag, Vs(P), Vs(T[0]), Vs(T[1]), Vs(T[2]), D);

            Float thickness  = VThickness(P) + VThickness(T[0]);
            Float thickness2 = thickness * thickness;

            if(D <= thickness2)
                return true;
            return false;
        }
    };

    struct SimplicialSurfaceDistanceCheck_EE_LeafPred
    {
        cuda_tool::CBufferView<Vector3>  Vs;
        cuda_tool::CBufferView<Float>    VThickness;
        cuda_tool::CBufferView<Vector2i> Es;
        cuda_tool::CBufferView<IndexT>   CIds;
        cuda_tool::CBufferView<IndexT>   SCIds;
        cuda_tool::CBufferView<IndexT>   SelfCollision;
        cuda_tool::CDense2D<IndexT>      ContactTable;
        cuda_tool::CDense2D<IndexT>      SubsceneTable;

        __device__ bool operator()(const InfoStacklessBVH::LeafPredInfo& info) const
        {
            IndexT   ei = info.i;
            IndexT   ej = info.j;
            Vector2i E0 = Es(ei);
            Vector2i E1 = Es(ej);

            if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0]
               || E0[1] == E1[1])
                return false;

            Vector4i scids = {
                SCIds(E0[0]), SCIds(E0[1]), SCIds(E1[0]), SCIds(E1[1])};
            if(!allow_EE_contact(SubsceneTable, scids))
                return false;

            Vector4i cids = {CIds(E0[0]), CIds(E0[1]), CIds(E1[0]), CIds(E1[1])};
            if(!allow_EE_contact(ContactTable, cids))
                return false;

            if(info.bid_i == info.bid_j
               && info.bid_i != static_cast<IndexT>(-1)
               && !SelfCollision(E0[0]))
                return false;

            auto ee_flag = distance::edge_edge_distance_flag(
                Vs(E0[0]), Vs(E0[1]), Vs(E1[0]), Vs(E1[1]));
            Float D;
            distance::edge_edge_distance2(
                ee_flag, Vs(E0[0]), Vs(E0[1]), Vs(E1[0]), Vs(E1[1]), D);

            Float thickness = VThickness(E0[0]) + VThickness(E1[0]);
            Float thickness2 = thickness * thickness;

            if(D <= thickness2)
                return true;
            return false;
        }
    };
}  // namespace

class SimplicialSurfaceDistanceCheck final : public BackendSanityChecker
{
  public:
    constexpr static U64 SanityCheckerUID = 3;
    using BackendSanityChecker::BackendSanityChecker;

    enum ViolationType : IndexT
    {
        PP = 0,
        PE = 1,
        PT = 2,
        EE = 3
    };

    struct ViolationInfo
    {
        IndexT   type              = 0;
        Vector2i primitive_indices = {-1, -1};
        Vector2i geo_ids           = {-1, -1};
        Vector2i obj_ids           = {-1, -1};
        Vector2  distance          = {0, 0};
    };

    struct Vector2iLess
    {
        bool operator()(const Vector2i& lhs, const Vector2i& rhs) const
        {
            return lhs[0] < rhs[0] || (lhs[0] == rhs[0] && lhs[1] < rhs[1]);
        }
    };

    struct ViolationResult
    {
        bool                                  is_too_close = false;
        vector<ViolationInfo>                 violations;
        map<Vector2i, Vector2i, Vector2iLess> close_geo_ids;
        map<Vector2i, Vector2, Vector2iLess>  close_geo_distances;
        IndexT                                total_violations = 0;
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
                              span<const IndexT>   h_vert_geo_ids,
                              span<const IndexT>   h_vert_object_ids,
                              span<const IndexT>   h_contact_mask,
                              SizeT                contact_element_count,
                              span<const IndexT>   h_subscene_mask,
                              SizeT                subscene_element_count)
        {
            using namespace cuda_tool;

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
            {
                auto k = SimplicialSurfaceDistanceCheck_build_point_aabbs_kernel;
                int  n = (int)num_verts;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        positions.cview(),
                        thickness.cview(),
                        d_hat.cview(),
                        point_aabbs.view(),
                        n);
                }
            }

            DeviceBuffer<AABB> codim_point_aabbs(num_codim);
            if(num_codim > 0)
            {
                auto k = SimplicialSurfaceDistanceCheck_build_codim_point_aabbs_kernel;
                int  n = (int)num_codim;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        positions.cview(),
                        thickness.cview(),
                        d_hat.cview(),
                        codim_indices.cview(),
                        codim_point_aabbs.view(),
                        n);
                }
            }

            DeviceBuffer<AABB> edge_aabbs(num_edges);
            if(num_edges > 0)
            {
                auto k = SimplicialSurfaceDistanceCheck_build_edge_aabbs_kernel;
                int  n = (int)num_edges;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        positions.cview(),
                        edges.cview(),
                        thickness.cview(),
                        d_hat.cview(),
                        edge_aabbs.view(),
                        n);
                }
            }

            DeviceBuffer<AABB> tri_aabbs(num_tris);
            if(num_tris > 0)
            {
                auto k = SimplicialSurfaceDistanceCheck_build_tri_aabbs_kernel;
                int  n = (int)num_tris;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        positions.cview(),
                        triangles.cview(),
                        thickness.cview(),
                        d_hat.cview(),
                        tri_aabbs.view(),
                        n);
                }
            }

            auto& point_bids = vert_bids;
            auto& point_cids = vert_cids;

            DeviceBuffer<IndexT> codim_bids(num_codim);
            DeviceBuffer<IndexT> codim_cids(num_codim);
            if(num_codim > 0)
            {
                auto k = SimplicialSurfaceDistanceCheck_gather_codim_bids_cids_kernel;
                int  n = (int)num_codim;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        codim_indices.cview(),
                        vert_bids.cview(),
                        vert_cids.cview(),
                        codim_bids.view(),
                        codim_cids.view(),
                        n);
                }
            }

            DeviceBuffer<IndexT> edge_bids(num_edges);
            DeviceBuffer<IndexT> edge_cids(num_edges);
            if(num_edges > 0)
            {
                auto k = SimplicialSurfaceDistanceCheck_gather_edge_bids_cids_kernel;
                int  n = (int)num_edges;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        edges.cview(),
                        vert_bids.cview(),
                        vert_cids.cview(),
                        edge_bids.view(),
                        edge_cids.view(),
                        n);
                }
            }

            DeviceBuffer<IndexT> tri_bids(num_tris);
            DeviceBuffer<IndexT> tri_cids(num_tris);
            if(num_tris > 0)
            {
                auto k = SimplicialSurfaceDistanceCheck_gather_tri_bids_cids_kernel;
                int  n = (int)num_tris;
                if(n > 0)
                {
                    k<<<best_grid_dim(n, k), best_block_dim(k), 0, nullptr>>>(
                        triangles.cview(),
                        vert_bids.cview(),
                        vert_cids.cview(),
                        tri_bids.view(),
                        tri_cids.view(),
                        n);
                }
            }

            SizeT                  CN = contact_element_count;
            DeviceBuffer2D<IndexT> cmts;
            cmts.resize(Extent2D{CN, CN});
            cmts.view().copy_from(h_contact_mask.data());

            SizeT                  SN = subscene_element_count;
            DeviceBuffer2D<IndexT> scmts;
            scmts.resize(Extent2D{SN, SN});
            scmts.view().copy_from(h_subscene_mask.data());

            auto ContactMask = cmts.viewer();

            InfoStacklessBVH::QueryBuffer pp_qbuf, pe_qbuf, pt_qbuf, ee_qbuf;

            auto CIds          = vert_cids.cviewer();
            auto SCIds         = vert_scids.cviewer();
            auto SelfCollision = self_collision.cviewer();
            auto ContactTable  = cmts.cviewer();
            auto SubsceneTable = scmts.cviewer();

            // Phase 1: CodimP vs AllP
            if(num_codim > 0 && num_verts > 0)
            {
                InfoStacklessBVH point_bvh;
                point_bvh.build(
                    point_aabbs.view(), point_bids.view(), point_cids.view());

                auto Vs         = positions.cviewer();
                auto VThickness = thickness.cviewer();
                auto CodimPs    = codim_indices.cviewer();

                point_bvh.query(codim_point_aabbs.view(),
                                codim_bids.view(),
                                codim_cids.view(),
                                cmts.view(),
                                SimplicialSurfaceDistanceCheck_NodePred{ContactMask},
                                SimplicialSurfaceDistanceCheck_PP_LeafPred{Vs,
                                                                           VThickness,
                                                                           CodimPs,
                                                                           CIds,
                                                                           SCIds,
                                                                           SelfCollision,
                                                                           ContactTable,
                                                                           SubsceneTable},
                                pp_qbuf);
            }

            // Phase 2: CodimP vs AllE
            if(num_codim > 0 && num_edges > 0)
            {
                InfoStacklessBVH edge_bvh;
                edge_bvh.build(edge_aabbs.view(), edge_bids.view(), edge_cids.view());

                auto Vs         = positions.cviewer();
                auto VThickness = thickness.cviewer();
                auto Es         = edges.cviewer();
                auto CodimPs    = codim_indices.cviewer();

                edge_bvh.query(codim_point_aabbs.view(),
                               codim_bids.view(),
                               codim_cids.view(),
                               cmts.view(),
                               SimplicialSurfaceDistanceCheck_NodePred{ContactMask},
                               SimplicialSurfaceDistanceCheck_PE_LeafPred{Vs,
                                                                          VThickness,
                                                                          Es,
                                                                          CodimPs,
                                                                          CIds,
                                                                          SCIds,
                                                                          SelfCollision,
                                                                          ContactTable,
                                                                          SubsceneTable},
                               pe_qbuf);
            }

            // Phase 3: AllP vs AllT
            if(num_verts > 0 && num_tris > 0)
            {
                InfoStacklessBVH tri_bvh;
                tri_bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());

                auto Vs         = positions.cviewer();
                auto VThickness = thickness.cviewer();
                auto Fs         = triangles.cviewer();

                tri_bvh.query(point_aabbs.view(),
                              point_bids.view(),
                              point_cids.view(),
                              cmts.view(),
                              SimplicialSurfaceDistanceCheck_NodePred{ContactMask},
                              SimplicialSurfaceDistanceCheck_PT_LeafPred{Vs,
                                                                         VThickness,
                                                                         Fs,
                                                                         CIds,
                                                                         SCIds,
                                                                         SelfCollision,
                                                                         ContactTable,
                                                                         SubsceneTable},
                              pt_qbuf);
            }

            // Phase 4: AllE vs AllE
            if(num_edges > 1)
            {
                InfoStacklessBVH edge_bvh;
                edge_bvh.build(edge_aabbs.view(), edge_bids.view(), edge_cids.view());

                auto Vs         = positions.cviewer();
                auto VThickness = thickness.cviewer();
                auto Es         = edges.cviewer();

                edge_bvh.detect(cmts.view(),
                                SimplicialSurfaceDistanceCheck_NodePred{ContactMask},
                                SimplicialSurfaceDistanceCheck_EE_LeafPred{Vs,
                                                                           VThickness,
                                                                           Es,
                                                                           CIds,
                                                                           SCIds,
                                                                           SelfCollision,
                                                                           ContactTable,
                                                                           SubsceneTable},
                                ee_qbuf);
            }

            auto copy_pairs = [](InfoStacklessBVH::QueryBuffer& qbuf) -> vector<Vector2i>
            {
                vector<Vector2i> pairs(qbuf.size());
                if(!pairs.empty())
                    qbuf.view().copy_to(pairs.data());
                return pairs;
            };

            auto pp_pairs = copy_pairs(pp_qbuf);
            auto pe_pairs = copy_pairs(pe_qbuf);
            auto pt_pairs = copy_pairs(pt_qbuf);
            auto ee_pairs = copy_pairs(ee_qbuf);

            ViolationResult result;
            result.total_violations = static_cast<IndexT>(
                pp_pairs.size() + pe_pairs.size() + pt_pairs.size() + ee_pairs.size());
            result.is_too_close = result.total_violations > 0;
            result.violations.reserve(result.total_violations);

            for(const auto& p : pp_pairs)
            {
                IndexT CodimP = CodimIndices[p[0]];
                Float  D;
                distance::point_point_distance2(Vs[CodimP], Vs[p[1]], D);

                Float thickness = h_thickness_span[CodimP] + h_thickness_span[p[1]];
                Float thickness2 = thickness * thickness;

                result.violations.push_back(
                    {PP,
                     p,
                     {h_vert_geo_ids[CodimP], h_vert_geo_ids[p[1]]},
                     {h_vert_object_ids[CodimP], h_vert_object_ids[p[1]]},
                     {D, thickness2}});
            }

            for(const auto& p : pe_pairs)
            {
                IndexT   CodimP = CodimIndices[p[0]];
                Vector2i E      = Es[p[1]];

                auto pe_flag =
                    distance::point_edge_distance_flag(Vs[CodimP], Vs[E[0]], Vs[E[1]]);
                Float D;
                distance::point_edge_distance2(pe_flag, Vs[CodimP], Vs[E[0]], Vs[E[1]], D);

                Float thickness = h_thickness_span[CodimP] + h_thickness_span[E[0]];
                Float thickness2 = thickness * thickness;

                result.violations.push_back(
                    {PE,
                     p,
                     {h_vert_geo_ids[CodimP], h_vert_geo_ids[E[0]]},
                     {h_vert_object_ids[CodimP], h_vert_object_ids[E[0]]},
                     {D, thickness2}});
            }

            for(const auto& p : pt_pairs)
            {
                IndexT   P = p[0];
                Vector3i F = Fs[p[1]];

                auto pt_flag = distance::point_triangle_distance_flag(
                    Vs[P], Vs[F[0]], Vs[F[1]], Vs[F[2]]);
                Float D;
                distance::point_triangle_distance2(
                    pt_flag, Vs[P], Vs[F[0]], Vs[F[1]], Vs[F[2]], D);

                Float thickness  = h_thickness_span[P] + h_thickness_span[F[0]];
                Float thickness2 = thickness * thickness;

                result.violations.push_back(
                    {PT,
                     p,
                     {h_vert_geo_ids[P], h_vert_geo_ids[F[0]]},
                     {h_vert_object_ids[P], h_vert_object_ids[F[0]]},
                     {D, thickness2}});
            }

            for(const auto& p : ee_pairs)
            {
                Vector2i E0 = Es[p[0]];
                Vector2i E1 = Es[p[1]];

                auto ee_flag = distance::edge_edge_distance_flag(
                    Vs[E0[0]], Vs[E0[1]], Vs[E1[0]], Vs[E1[1]]);
                Float D;
                distance::edge_edge_distance2(
                    ee_flag, Vs[E0[0]], Vs[E0[1]], Vs[E1[0]], Vs[E1[1]], D);

                Float thickness = h_thickness_span[E0[0]] + h_thickness_span[E1[0]];
                Float thickness2 = thickness * thickness;

                result.violations.push_back(
                    {EE,
                     p,
                     {h_vert_geo_ids[E0[0]], h_vert_geo_ids[E1[0]]},
                     {h_vert_object_ids[E0[0]], h_vert_object_ids[E1[0]]},
                     {D, thickness2}});
            }

            auto set_geo_distance = [&](const Vector2i& geo_ids, const Vector2& distance)
            {
                if(auto it = result.close_geo_distances.find(geo_ids);
                   it == result.close_geo_distances.end())
                {
                    result.close_geo_distances[geo_ids] = distance;
                }
                else if(distance[0] < it->second[0])
                {
                    it->second = distance;
                }
            };

            for(auto info : result.violations)
            {
                if(info.geo_ids[0] > info.geo_ids[1])
                {
                    std::swap(info.geo_ids[0], info.geo_ids[1]);
                    std::swap(info.obj_ids[0], info.obj_ids[1]);
                }

                result.close_geo_ids[info.geo_ids] = info.obj_ids;
                set_geo_distance(info.geo_ids, info.distance);
            }

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

        const geometry::SimplicialComplex& scene_surface = ctx.scene_simplicial_surface();

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

        auto attr_v_geo_ids =
            scene_surface.vertices().find<IndexT>("sanity_check/geometry_id");
        UIPC_ASSERT(attr_v_geo_ids, "`sanity_check/geometry_id` is not found in scene surface");
        auto VGeoIds = attr_v_geo_ids->view();

        auto attr_v_object_id =
            scene_surface.vertices().find<IndexT>("sanity_check/object_id");
        UIPC_ASSERT(attr_v_object_id, "`sanity_check/object_id` is not found in scene surface");
        auto VObjectIds = attr_v_object_id->view();

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
                h_contact_mask[i * CN + j] =
                    contact_tabular.at(i, j).is_enabled() ? 1 : 0;

        auto& subscene_tabular = ctx.subscene_tabular();
        SizeT SN               = subscene_tabular.element_count();

        vector<IndexT> h_subscene_mask(SN * SN);
        for(IndexT i = 0; i < (IndexT)SN; ++i)
            for(IndexT j = 0; j < (IndexT)SN; ++j)
                h_subscene_mask[i * SN + j] =
                    subscene_tabular.at(i, j).is_enabled() ? 1 : 0;

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
                         span<const IndexT>{VGeoIds.data(), VGeoIds.size()},
                         span<const IndexT>{VObjectIds.data(), VObjectIds.size()},
                         span<const IndexT>{h_contact_mask},
                         CN,
                         span<const IndexT>{h_subscene_mask},
                         SN);

        if(!result.is_too_close)
            return SanityCheckResult::Success;

        SizeT num_edges = Es.size();
        SizeT num_tris  = Fs.size();

        vector<IndexT> vertex_too_close(num_verts, 0);
        vector<IndexT> edge_too_close(num_edges, 0);
        vector<IndexT> tri_too_close(num_tris, 0);

        auto mark_vert = [&](IndexT v) { vertex_too_close[v] = 1; };

        for(auto& info : result.violations)
        {
            switch(info.type)
            {
                case PP: {
                    IndexT CodimP = h_codim_indices[info.primitive_indices[0]];
                    mark_vert(CodimP);
                    mark_vert(info.primitive_indices[1]);
                    break;
                }
                case PE: {
                    IndexT CodimP = h_codim_indices[info.primitive_indices[0]];
                    mark_vert(CodimP);
                    Vector2i E = Es[info.primitive_indices[1]];
                    mark_vert(E[0]);
                    mark_vert(E[1]);
                    edge_too_close[info.primitive_indices[1]] = 1;
                    break;
                }
                case PT: {
                    mark_vert(info.primitive_indices[0]);
                    Vector3i F = Fs[info.primitive_indices[1]];
                    mark_vert(F[0]);
                    mark_vert(F[1]);
                    mark_vert(F[2]);
                    tri_too_close[info.primitive_indices[1]] = 1;
                    break;
                }
                case EE: {
                    Vector2i E0 = Es[info.primitive_indices[0]];
                    Vector2i E1 = Es[info.primitive_indices[1]];
                    mark_vert(E0[0]);
                    mark_vert(E0[1]);
                    mark_vert(E1[0]);
                    mark_vert(E1[1]);
                    edge_too_close[info.primitive_indices[0]] = 1;
                    edge_too_close[info.primitive_indices[1]] = 1;
                    break;
                }
                default:
                    UIPC_ASSERT(false, "Unknown violation type: {}", info.type);
            }
        }

        ::uipc::backend::SanityCheckMessageVisitor scmv{msg};
        auto&                                      buffer = scmv.message();

        for(auto& [GeoIds, ObjIds] : result.close_geo_ids)
        {
            auto obj_0 = find_object(ObjIds[0]);
            auto obj_1 = find_object(ObjIds[1]);

            UIPC_ASSERT(obj_0 != nullptr, "Object[{}] not found", ObjIds[0]);
            UIPC_ASSERT(obj_1 != nullptr, "Object[{}] not found", ObjIds[1]);

            std::string name_0{obj_0->name()};
            std::string name_1{obj_1->name()};

            fmt::format_to(std::back_inserter(buffer),
                           "Geometry({}) in Object[{}({})] is too close (distance={}, thickness={}) to Geometry({}) in "
                           "Object[{}({})]\n",
                           GeoIds[0],
                           name_0,
                           obj_0->id(),
                           std::sqrt(result.close_geo_distances[GeoIds][0]),
                           std::sqrt(result.close_geo_distances[GeoIds][1]),
                           GeoIds[1],
                           name_1,
                           obj_1->id());
        }

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
