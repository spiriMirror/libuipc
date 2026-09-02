#include <collision_detection/filters/info_stackless_bvh_simplex_trajectory_filter.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>
#include <utils/simplex_contact_mask_utils.h>
#include <uipc/common/zip.h>
#include <utils/primitive_d_hat.h>
#include <array>
#include <cstdio>

namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo          = false;
constexpr bool PrintKernelZeroDistance = false;

namespace
{
    constexpr SizeT max_iter = 1000;

    constexpr Float large_enough_toi = 1.1;

    /****************************************************
    *                   Broad Phase
    ****************************************************/

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k1_kernel(
        cuda_tool::CBufferView<IndexT>  codimVs,
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::BufferView<AABB>     aabbs,
        cuda_tool::CBufferView<IndexT>  v2bs,
        cuda_tool::CBufferView<IndexT>  contact_ids,
        cuda_tool::BufferView<IndexT>   bids,
        cuda_tool::BufferView<IndexT>   cids,
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto vI = codimVs(i);

        Float thickness       = thicknesses(vI);
        Float d_hat_expansion = point_dcd_expansion(d_hats(vI));

        const auto& pos   = Ps(vI);
        Vector3     pos_t = pos + dxs(vI) * alpha;

        AABB aabb;
        aabb.extend(pos.cast<float>()).extend(pos_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(vI);
        cids(i)  = contact_ids(vI);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k2_kernel(
        cuda_tool::CBufferView<IndexT>  Vs,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::BufferView<AABB>     aabbs,
        cuda_tool::CBufferView<IndexT>  v2bs,
        cuda_tool::CBufferView<IndexT>  contact_ids,
        cuda_tool::BufferView<IndexT>   bids,
        cuda_tool::BufferView<IndexT>   cids,
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto vI = Vs(i);

        Float thickness       = thicknesses(vI);
        Float d_hat_expansion = point_dcd_expansion(d_hats(vI));

        const auto& pos   = Ps(vI);
        Vector3     pos_t = pos + dxs(vI) * alpha;

        AABB aabb;
        aabb.extend(pos.cast<float>()).extend(pos_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(vI);
        cids(i)  = contact_ids(vI);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k3_kernel(
        cuda_tool::CBufferView<Vector2i> Es,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::BufferView<AABB>      aabbs,
        cuda_tool::CBufferView<IndexT>   v2bs,
        cuda_tool::CBufferView<IndexT>   contact_ids,
        cuda_tool::BufferView<IndexT>    bids,
        cuda_tool::BufferView<IndexT>    cids,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto eI = Es(i);

        Float thickness = edge_thickness(thicknesses(eI[0]), thicknesses(eI[1]));
        Float d_hat_expansion = edge_dcd_expansion(d_hats(eI[0]), d_hats(eI[1]));

        const auto& pos0   = Ps(eI[0]);
        const auto& pos1   = Ps(eI[1]);
        Vector3     pos0_t = pos0 + dxs(eI[0]) * alpha;
        Vector3     pos1_t = pos1 + dxs(eI[1]) * alpha;

        AABB aabb;

        aabb.extend(pos0.cast<float>())
            .extend(pos1.cast<float>())
            .extend(pos0_t.cast<float>())
            .extend(pos1_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(eI[0]);
        cids(i)  = contact_ids(eI[0]);
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_detect_k4_kernel(
        cuda_tool::CBufferView<Vector3i> Fs,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::BufferView<AABB>      aabbs,
        cuda_tool::CBufferView<IndexT>   v2bs,
        cuda_tool::CBufferView<IndexT>   contact_ids,
        cuda_tool::BufferView<IndexT>    bids,
        cuda_tool::BufferView<IndexT>    cids,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto fI = Fs(i);

        Float thickness = triangle_thickness(
            thicknesses(fI[0]), thicknesses(fI[1]), thicknesses(fI[2]));
        Float d_hat_expansion =
            triangle_dcd_expansion(d_hats(fI[0]), d_hats(fI[1]), d_hats(fI[2]));

        const auto& pos0   = Ps(fI[0]);
        const auto& pos1   = Ps(fI[1]);
        const auto& pos2   = Ps(fI[2]);
        Vector3     pos0_t = pos0 + dxs(fI[0]) * alpha;
        Vector3     pos1_t = pos1 + dxs(fI[1]) * alpha;
        Vector3     pos2_t = pos2 + dxs(fI[2]) * alpha;

        AABB aabb;

        aabb.extend(pos0.cast<float>())
            .extend(pos1.cast<float>())
            .extend(pos2.cast<float>())
            .extend(pos0_t.cast<float>())
            .extend(pos1_t.cast<float>())
            .extend(pos2_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
        bids(i)  = v2bs(fI[0]);
        cids(i)  = contact_ids(fI[0]);
    }

    struct InfoStacklessBVHSimplexTrajectoryFilter_detect_node_pred
    {
        cuda_tool::CBufferView<IndexT> body_self_collision;
        cuda_tool::CDense2D<IndexT>    cmts;

        __device__ bool operator()(InfoStacklessBVH::NodePredInfo info) const
        {
            constexpr IndexT invalid = static_cast<IndexT>(-1);
            auto             qbid    = info.query_bid;
            auto             qcid    = info.query_cid;
            bool bid_cull = info.node_bid != invalid && qbid != invalid
                            && qbid == info.node_bid && !body_self_collision(qbid);
            bool cid_cull = info.node_cid != invalid && qcid != invalid
                            && !cmts(qcid, info.node_cid);
            return !(bid_cull || cid_cull);
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_detect_AllP_CodimP_pred
    {
        cuda_tool::CBufferView<IndexT>  Vs;
        cuda_tool::CBufferView<IndexT>  codimVs;
        cuda_tool::CBufferView<Vector3> Ps;
        cuda_tool::CBufferView<Vector3> dxs;
        cuda_tool::CBufferView<Float>   thicknesses;
        cuda_tool::CBufferView<IndexT>  dimensions;
        cuda_tool::CBufferView<IndexT>  contact_element_ids;
        cuda_tool::CDense2D<IndexT>     contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>  subscene_element_ids;
        cuda_tool::CDense2D<IndexT>     subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>  body_self_collision;
        cuda_tool::CBufferView<Float>   d_hats;
        Float                           alpha;

        __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
        {
            auto        i      = info.i;
            auto        j      = info.j;
            const auto& V      = Vs(i);
            const auto& codimV = codimVs(j);

            Vector2i cids = {contact_element_ids(V), contact_element_ids(codimV)};
            Vector2i scids = {subscene_element_ids(V), subscene_element_ids(codimV)};

            if(!allow_PP_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PP_contact(contact_mask_tabular, cids))
                return false;

            bool V_is_codim = dimensions(V) <= 2;

            if(V_is_codim && V >= codimV)
                return false;

            if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
               && !body_self_collision(info.bid_i))
                return false;

            Vector3 P0 = Ps(V);
            Vector3 P1 = Ps(codimV);

            Float thickness = PP_thickness(thicknesses(V), thicknesses(codimV));
            Float d_hat     = PP_d_hat(d_hats(V), d_hats(codimV));

            Float expand = d_hat + thickness;

            if(alpha == 0.0)
            {
                // DCD detect pass: exact distance test. The conservative
                // ccd_broadphase degenerates to a per-axis box test at
                // alpha==0 and passes ~50x more candidates than the exact
                // activation filter downstream re-keeps anyway. Pairs with
                // D2 <= thickness^2 are kept so the thickness-violation
                // handling in filter_active still sees them.
                Float D2;
                distance::point_point_distance2(P0, P1, D2);
                return D2 < expand * expand;
            }

            Vector3 dP0 = alpha * dxs(V);
            Vector3 dP1 = alpha * dxs(codimV);

            if(!distance::point_point_ccd_broadphase(P0, P1, dP0, dP1, expand))
                return false;

            return true;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_detect_CodimP_AllE_pred
    {
        cuda_tool::CBufferView<IndexT>   codimVs;
        cuda_tool::CBufferView<Vector2i> Es;
        cuda_tool::CBufferView<Vector3>  Ps;
        cuda_tool::CBufferView<Vector3>  dxs;
        cuda_tool::CBufferView<Float>    thicknesses;
        cuda_tool::CBufferView<IndexT>   contact_element_ids;
        cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>   subscene_element_ids;
        cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>   body_self_collision;
        cuda_tool::CBufferView<Float>    d_hats;
        Float                            alpha;

        __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
        {
            auto        i      = info.i;
            auto        j      = info.j;
            const auto& codimV = codimVs(i);
            const auto& E      = Es(j);

            Vector3i cids = {contact_element_ids(codimV),
                             contact_element_ids(E[0]),
                             contact_element_ids(E[1])};

            Vector3i scids = {subscene_element_ids(codimV),
                              subscene_element_ids(E[0]),
                              subscene_element_ids(E[1])};

            if(!allow_PE_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PE_contact(contact_mask_tabular, cids))
                return false;

            if(E[0] == codimV || E[1] == codimV)
                return false;

            if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
               && !body_self_collision(info.bid_i))
                return false;

            Vector3 E0 = Ps(E[0]);
            Vector3 E1 = Ps(E[1]);

            Vector3 P = Ps(codimV);

            Float thickness =
                PE_thickness(thicknesses(codimV), thicknesses(E[0]), thicknesses(E[1]));
            Float d_hat = PE_d_hat(d_hats(codimV), d_hats(E[0]), d_hats(E[1]));

            Float expand = d_hat + thickness;

            if(alpha == 0.0)
            {
                // DCD detect pass: exact distance test (see PP pred)
                Float    D2;
                Vector3i flag = distance::point_edge_distance_flag(P, E0, E1);
                distance::point_edge_distance2(flag, P, E0, E1, D2);
                return D2 < expand * expand;
            }

            Vector3 dE0 = alpha * dxs(E[0]);
            Vector3 dE1 = alpha * dxs(E[1]);
            Vector3 dP  = alpha * dxs(codimV);

            if(!distance::point_edge_ccd_broadphase(P, E0, E1, dP, dE0, dE1, expand))
                return false;

            return true;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_detect_AllE_AllE_pred
    {
        cuda_tool::CBufferView<Vector2i> Es;
        cuda_tool::CBufferView<Vector3>  Ps;
        cuda_tool::CBufferView<Vector3>  dxs;
        cuda_tool::CBufferView<Float>    thicknesses;
        cuda_tool::CBufferView<IndexT>   contact_element_ids;
        cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>   subscene_element_ids;
        cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>   body_self_collision;
        cuda_tool::CBufferView<Float>    d_hats;
        Float                            alpha;

        __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
        {
            auto        i  = info.i;
            auto        j  = info.j;
            const auto& E0 = Es(i);
            const auto& E1 = Es(j);

            Vector4i cids = {contact_element_ids(E0[0]),
                             contact_element_ids(E0[1]),
                             contact_element_ids(E1[0]),
                             contact_element_ids(E1[1])};

            Vector4i scids = {subscene_element_ids(E0[0]),
                              subscene_element_ids(E0[1]),
                              subscene_element_ids(E1[0]),
                              subscene_element_ids(E1[1])};

            if(!allow_EE_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_EE_contact(contact_mask_tabular, cids))
                return false;

            if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                return false;

            if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
               && !body_self_collision(info.bid_i))
                return false;

            Vector3 E0_0 = Ps(E0[0]);
            Vector3 E0_1 = Ps(E0[1]);

            Vector3 E1_0 = Ps(E1[0]);
            Vector3 E1_1 = Ps(E1[1]);

            Float thickness = EE_thickness(thicknesses(E0[0]),
                                           thicknesses(E0[1]),
                                           thicknesses(E1[0]),
                                           thicknesses(E1[1]));

            Float d_hat =
                EE_d_hat(d_hats(E0[0]), d_hats(E0[1]), d_hats(E1[0]), d_hats(E1[1]));

            Float expand = d_hat + thickness;

            if(alpha == 0.0)
            {
                // DCD detect pass: exact distance test (see PP pred); the
                // plain EE distance is a correct superset of the mollified
                // degenerate handling in filter_active
                Float D2;
                Vector4i flag = distance::edge_edge_distance_flag(E0_0, E0_1, E1_0, E1_1);
                distance::edge_edge_distance2(flag, E0_0, E0_1, E1_0, E1_1, D2);
                return D2 < expand * expand;
            }

            Vector3 dE0_0 = alpha * dxs(E0[0]);
            Vector3 dE0_1 = alpha * dxs(E0[1]);
            Vector3 dE1_0 = alpha * dxs(E1[0]);
            Vector3 dE1_1 = alpha * dxs(E1[1]);

            if(!distance::edge_edge_ccd_broadphase(
                   E0_0, E0_1, E1_0, E1_1, dE0_0, dE0_1, dE1_0, dE1_1, expand))
                return false;

            return true;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_detect_AllP_AllT_pred
    {
        cuda_tool::CBufferView<IndexT>   Vs;
        cuda_tool::CBufferView<Vector3i> Fs;
        cuda_tool::CBufferView<Vector3>  Ps;
        cuda_tool::CBufferView<Vector3>  dxs;
        cuda_tool::CBufferView<Float>    thicknesses;
        cuda_tool::CBufferView<IndexT>   contact_element_ids;
        cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>   subscene_element_ids;
        cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>   body_self_collision;
        cuda_tool::CBufferView<Float>    d_hats;
        Float                            alpha;

        __device__ bool operator()(InfoStacklessBVH::LeafPredInfo info) const
        {
            auto i = info.i;
            auto j = info.j;
            auto V = Vs(i);
            auto F = Fs(j);

            Vector4i cids = {contact_element_ids(V),
                             contact_element_ids(F[0]),
                             contact_element_ids(F[1]),
                             contact_element_ids(F[2])};

            Vector4i scids = {subscene_element_ids(V),
                              subscene_element_ids(F[0]),
                              subscene_element_ids(F[1]),
                              subscene_element_ids(F[2])};

            if(!allow_PT_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PT_contact(contact_mask_tabular, cids))
                return false;

            if(F[0] == V || F[1] == V || F[2] == V)
                return false;

            if(info.bid_i == info.bid_j && info.bid_i != static_cast<IndexT>(-1)
               && !body_self_collision(info.bid_i))
                return false;

            Vector3 P = Ps(V);

            Vector3 F0 = Ps(F[0]);
            Vector3 F1 = Ps(F[1]);
            Vector3 F2 = Ps(F[2]);

            Float thickness = PT_thickness(thicknesses(V),
                                           thicknesses(F[0]),
                                           thicknesses(F[1]),
                                           thicknesses(F[2]));

            Float d_hat = PT_d_hat(d_hats(V), d_hats(F[0]), d_hats(F[1]), d_hats(F[2]));

            Float expand = d_hat + thickness;

            if(alpha == 0.0)
            {
                // DCD detect pass: exact distance test (see PP pred)
                Float D2;
                Vector4i flag = distance::point_triangle_distance_flag(P, F0, F1, F2);
                distance::point_triangle_distance2(flag, P, F0, F1, F2, D2);
                return D2 < expand * expand;
            }

            Vector3 dP = alpha * dxs(V);

            Vector3 dF0 = alpha * dxs(F[0]);
            Vector3 dF1 = alpha * dxs(F[1]);
            Vector3 dF2 = alpha * dxs(F[2]);

            if(!distance::point_triangle_ccd_broadphase(P, F0, F1, F2, dP, dF0, dF1, dF2, expand))
                return false;

            return true;
        }
    };

    /****************************************************
    *                   Filter Active
    ****************************************************/

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k1_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> PCodimP_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::CBufferView<Float>    d_hats,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);

        Vector2i indices = PCodimP_pairs(i);

        IndexT P0 = surf_vertices(indices(0));
        IndexT P1 = codim_vertices(indices(1));

        const auto& V0 = positions(P0);
        const auto& V1 = positions(P1);

        Float thickness = PP_thickness(thicknesses(P0), thicknesses(P1));
        Float d_hat     = PP_d_hat(d_hats(P0), d_hats(P1));

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_point_distance2(V0, V1, D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][PP][low-dist] i=%d P=(%d,%d) D=%e range=(%e,%e) "
                    "thickness=%e d_hat=%e\n",
                    i,
                    P0,
                    P1,
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }

        UIPC_KERNEL_ASSERT(D > range.x(),
                           "Thickness Violated! D(%f) should be > D_range.x(%f), "
                           "P=(%d,%d), thickness=%f, d_hat=%f",
                           D,
                           range.x(),
                           P0,
                           P1,
                           thickness,
                           d_hat);
        if(!is_active_D(range, D))
            return;

        PP = {P0, P1};
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k2_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> CodimP_AllE_pairs,
        cuda_tool::CBufferView<IndexT>   codim_veritces,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::BufferView<Vector3i>  temp_PEs,
        cuda_tool::CBufferView<Float>    d_hats,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);
        auto& PE = temp_PEs(i);
        PE.setConstant(-1);

        Vector2i indices = CodimP_AllE_pairs(i);
        IndexT   V       = codim_veritces(indices(0));
        Vector2i E       = surf_edges(indices(1));

        Vector3i vIs = {V, E(0), E(1)};
        Vector3 Ps[] = {positions(vIs(0)), positions(vIs(1)), positions(vIs(2))};

        Float thickness =
            PE_thickness(thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));

        Float d_hat = PE_d_hat(d_hats(V), d_hats(E(0)), d_hats(E(1)));

        Vector3i flag = distance::point_edge_distance_flag(Ps[0], Ps[1], Ps[2]);

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_edge_distance2(flag, Ps[0], Ps[1], Ps[2], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][PE][low-dist] i=%d V-E=(%d,%d,%d) flag=(%d,%d,%d) "
                    "D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                    i,
                    vIs(0),
                    vIs(1),
                    vIs(2),
                    flag(0),
                    flag(1),
                    flag(2),
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }

        UIPC_KERNEL_ASSERT(D > range.x(),
                           "Thickness Violated! D(%f) should be > D_range.x(%f), "
                           "V-E=(%d,%d,%d), flag=(%d,%d,%d), thickness=%f, d_hat=%f",
                           D,
                           range.x(),
                           vIs(0),
                           vIs(1),
                           vIs(2),
                           flag(0),
                           flag(1),
                           flag(2),
                           thickness,
                           d_hat);
        if(!is_active_D(range, D))
            return;

        Vector3i offsets;
        auto     dim = distance::degenerate_point_edge(flag, offsets);

        switch(dim)
        {
            case 2: {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                PP        = {V0, V1};
            }
            break;
            case 3: {
                PE = vIs;
            }
            break;
            default: {
                UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
            }
            break;
        }
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k3_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector2i> PT_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Vector3i> surf_triangles,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::BufferView<Vector3i>  temp_PEs,
        cuda_tool::BufferView<Vector4i>  temp_PTs,
        cuda_tool::CBufferView<Float>    d_hats,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);
        auto& PE = temp_PEs(i);
        PE.setConstant(-1);
        auto& PT = temp_PTs(i);
        PT.setConstant(-1);

        Vector2i indices = PT_pairs(i);
        IndexT   V       = surf_vertices(indices(0));
        Vector3i F       = surf_triangles(indices(1));

        Vector4i vIs  = {V, F(0), F(1), F(2)};
        Vector3  Ps[] = {
            positions(vIs(0)), positions(vIs(1)), positions(vIs(2)), positions(vIs(3))};

        Float thickness = PT_thickness(
            thicknesses(V), thicknesses(F(0)), thicknesses(F(1)), thicknesses(F(2)));

        Float d_hat = PT_d_hat(d_hats(V), d_hats(F(0)), d_hats(F(1)), d_hats(F(2)));

        Vector4i flag =
            distance::point_triangle_distance_flag(Ps[0], Ps[1], Ps[2], Ps[3]);

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_triangle_distance2(flag, Ps[0], Ps[1], Ps[2], Ps[3], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][PT][low-dist] i=%d V-F=(%d,%d,%d,%d) "
                    "flag=(%d,%d,%d,%d) D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                    i,
                    vIs(0),
                    vIs(1),
                    vIs(2),
                    vIs(3),
                    flag(0),
                    flag(1),
                    flag(2),
                    flag(3),
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }

        UIPC_KERNEL_ASSERT(
            D > 0.0, "D=%f, V F = (%d,%d,%d,%d)", D, vIs(0), vIs(1), vIs(2), vIs(3));

        UIPC_KERNEL_ASSERT(D > range.x(),
                           "Thickness Violated! D(%f) should be > D_range.x(%f), "
                           "V-F=(%d,%d,%d,%d), flag=(%d,%d,%d,%d), thickness=%f, d_hat=%f",
                           D,
                           range.x(),
                           vIs(0),
                           vIs(1),
                           vIs(2),
                           vIs(3),
                           flag(0),
                           flag(1),
                           flag(2),
                           flag(3),
                           thickness,
                           d_hat);
        if(!is_active_D(range, D))
            return;

        Vector4i offsets;
        auto     dim = distance::degenerate_point_triangle(flag, offsets);

        switch(dim)
        {
            case 2: {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                PP        = {V0, V1};
            }
            break;
            case 3: {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                IndexT V2 = vIs(offsets(2));
                PE        = {V0, V1, V2};
            }
            break;
            case 4: {
                PT = vIs;
            }
            break;
            default: {
                UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
            }
            break;
        }
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k4_kernel(
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3>  rest_positions,
        cuda_tool::CBufferView<Vector2i> EE_pairs,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::BufferView<Vector2i>  temp_PPs,
        cuda_tool::BufferView<Vector3i>  temp_PEs,
        cuda_tool::BufferView<Vector4i>  temp_EEs,
        cuda_tool::CBufferView<Float>    d_hats,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);
        auto& PE = temp_PEs(i);
        PE.setConstant(-1);
        auto& EE = temp_EEs(i);
        EE.setConstant(-1);

        Vector2i indices = EE_pairs(i);
        Vector2i E0      = surf_edges(indices(0));
        Vector2i E1      = surf_edges(indices(1));

        Vector4i vIs  = {E0(0), E0(1), E1(0), E1(1)};
        Vector3  Ps[] = {
            positions(vIs(0)), positions(vIs(1)), positions(vIs(2)), positions(vIs(3))};

        Float thickness = EE_thickness(thicknesses(E0(0)),
                                       thicknesses(E0(1)),
                                       thicknesses(E1(0)),
                                       thicknesses(E1(1)));

        Float d_hat =
            EE_d_hat(d_hats(E0(0)), d_hats(E0(1)), d_hats(E1(0)), d_hats(E1(1)));

        Vector2 range = D_range(thickness, d_hat);

        Vector4i flag = distance::edge_edge_distance_flag(Ps[0], Ps[1], Ps[2], Ps[3]);

        Float D;
        distance::edge_edge_distance2(flag, Ps[0], Ps[1], Ps[2], Ps[3], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf(
                    "[ISBVH][EE][low-dist] i=%d E-E=(%d,%d,%d,%d) "
                    "flag=(%d,%d,%d,%d) D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                    i,
                    vIs(0),
                    vIs(1),
                    vIs(2),
                    vIs(3),
                    flag(0),
                    flag(1),
                    flag(2),
                    flag(3),
                    D,
                    range.x(),
                    range.y(),
                    thickness,
                    d_hat);
            }
        }
        if(D <= range.x())
        {
            EE = vIs;
            return;
        }
        if(!is_active_D(range, D))
            return;

        Float eps_x;
        distance::edge_edge_mollifier_threshold(rest_positions(vIs(0)),
                                                rest_positions(vIs(1)),
                                                rest_positions(vIs(2)),
                                                rest_positions(vIs(3)),
                                                static_cast<Float>(1e-3),
                                                eps_x);

        if(distance::need_mollify(Ps[0], Ps[1], Ps[2], Ps[3], eps_x))
        {
            EE = vIs;
            return;
        }
        else
        {
            Vector4i offsets;
            auto     dim = distance::degenerate_edge_edge(flag, offsets);

            switch(dim)
            {
                case 2: {
                    IndexT V0 = vIs(offsets(0));
                    IndexT V1 = vIs(offsets(1));
                    PP        = {V0, V1};
                }
                break;
                case 3: {
                    IndexT V0 = vIs(offsets(0));
                    IndexT V1 = vIs(offsets(1));
                    IndexT V2 = vIs(offsets(2));
                    PE        = {V0, V1, V2};
                }
                break;
                case 4: {
                    EE = vIs;
                }
                break;
                default: {
                    UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
                }
                break;
            }
        }
    }

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PP_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector2i& PP) const
        {
            return PP(0) != -1;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PE_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector3i& PE) const
        {
            return PE(0) != -1;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PT_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector4i& PT) const
        {
            return PT(0) != -1;
        }
    };

    struct InfoStacklessBVHSimplexTrajectoryFilter_filter_active_EE_pred
    {
        CUB_RUNTIME_FUNCTION bool operator()(const Vector4i& EE) const
        {
            return EE(0) != -1;
        }
    };

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_collect_query_counts_kernel(
        cuda_tool::CVarView<IndexT>   allp_codimp_count,
        cuda_tool::CVarView<IndexT>   codimp_alle_count,
        cuda_tool::CVarView<IndexT>   alle_alle_count,
        cuda_tool::CVarView<IndexT>   allp_allt_count,
        cuda_tool::BufferView<IndexT> counts)
    {
        if(blockIdx.x != 0 || threadIdx.x != 0)
            return;
        counts(0) = *allp_codimp_count;
        counts(1) = *codimp_alle_count;
        counts(2) = *alle_alle_count;
        counts(3) = *allp_allt_count;
    }

    /****************************************************
    *                   Filter TOI
    ****************************************************/

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k1_kernel(
        cuda_tool::BufferView<Float>     PP_tois,
        cuda_tool::CBufferView<Vector2i> PCodimP_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto   indices = PCodimP_pairs(i);
        IndexT V0      = surf_vertices(indices(0));
        IndexT V1      = codim_vertices(indices(1));

        Float thickness = PP_thickness(thicknesses(V0), thicknesses(V1));
        Float d_hat     = PP_d_hat(d_hats(V0), d_hats(V1));

        Vector3 VP0  = positions(V0);
        Vector3 VP1  = positions(V1);
        Vector3 dVP0 = alpha * dxs(V0);
        Vector3 dVP1 = alpha * dxs(V1);

        Float toi = large_enough_toi;

        bool faraway =
            !distance::point_point_ccd_broadphase(VP0, VP1, dVP0, dVP1, d_hat + thickness);

        if(faraway)
        {
            PP_tois(i) = toi;
            return;
        }

        bool hit =
            distance::point_point_ccd(VP0, VP1, dVP0, dVP1, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PP_tois(i) = toi;
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k2_kernel(
        cuda_tool::BufferView<Float>     PE_tois,
        cuda_tool::CBufferView<Vector2i> CodimP_AllE_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto     indices = CodimP_AllE_pairs(i);
        IndexT   V       = codim_vertices(indices(0));
        Vector2i E       = surf_edges(indices(1));

        Float thickness =
            PE_thickness(thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));
        Float d_hat = PE_d_hat(d_hats(V), d_hats(E(0)), d_hats(E(1)));

        Vector3 VP  = Ps(V);
        Vector3 dVP = alpha * dxs(V);

        Vector3 EP0  = Ps(E[0]);
        Vector3 EP1  = Ps(E[1]);
        Vector3 dEP0 = alpha * dxs(E[0]);
        Vector3 dEP1 = alpha * dxs(E[1]);

        Float toi = large_enough_toi;

        bool faraway = !distance::point_edge_ccd_broadphase(
            VP, EP0, EP1, dVP, dEP0, dEP1, d_hat + thickness);

        if(faraway)
        {
            PE_tois(i) = toi;
            return;
        }

        bool hit = distance::point_edge_ccd(
            VP, EP0, EP1, dVP, dEP0, dEP1, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PE_tois(i) = toi;
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k3_kernel(
        cuda_tool::BufferView<Float>     PT_tois,
        cuda_tool::CBufferView<Vector2i> PT_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Vector3i> surf_triangles,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto     indices = PT_pairs(i);
        IndexT   V       = surf_vertices(indices(0));
        Vector3i F       = surf_triangles(indices(1));

        Float thickness = PT_thickness(
            thicknesses(V), thicknesses(F(0)), thicknesses(F(1)), thicknesses(F(2)));
        Float d_hat = PT_d_hat(d_hats(V), d_hats(F(0)), d_hats(F(1)), d_hats(F(2)));

        Vector3 VP  = Ps(V);
        Vector3 dVP = alpha * dxs(V);

        Vector3 FP0 = Ps(F[0]);
        Vector3 FP1 = Ps(F[1]);
        Vector3 FP2 = Ps(F[2]);

        Vector3 dFP0 = alpha * dxs(F[0]);
        Vector3 dFP1 = alpha * dxs(F[1]);
        Vector3 dFP2 = alpha * dxs(F[2]);

        Float toi = large_enough_toi;

        bool faraway = !distance::point_triangle_ccd_broadphase(
            VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, d_hat + thickness);

        if(faraway)
        {
            PT_tois(i) = toi;
            return;
        }

        bool hit = distance::point_triangle_ccd(
            VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PT_tois(i) = toi;
    }

    __global__ void InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k4_kernel(
        cuda_tool::BufferView<Float>     EE_tois,
        cuda_tool::CBufferView<Vector2i> EE_pairs,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            eta,
        Float                            alpha,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto     indices = EE_pairs(i);
        Vector2i E0      = surf_edges(indices(0));
        Vector2i E1      = surf_edges(indices(1));

        Float thickness = EE_thickness(thicknesses(E0(0)),
                                       thicknesses(E0(1)),
                                       thicknesses(E1(0)),
                                       thicknesses(E1(1)));

        Float d_hat =
            EE_d_hat(d_hats(E0(0)), d_hats(E0(1)), d_hats(E1(0)), d_hats(E1(1)));

        Vector3 EP0  = Ps(E0[0]);
        Vector3 EP1  = Ps(E0[1]);
        Vector3 dEP0 = alpha * dxs(E0[0]);
        Vector3 dEP1 = alpha * dxs(E0[1]);

        Vector3 EP2  = Ps(E1[0]);
        Vector3 EP3  = Ps(E1[1]);
        Vector3 dEP2 = alpha * dxs(E1[0]);
        Vector3 dEP3 = alpha * dxs(E1[1]);

        Float toi = large_enough_toi;

        bool faraway = !distance::edge_edge_ccd_broadphase(
            EP0, EP1, EP2, EP3, dEP0, dEP1, dEP2, dEP3, d_hat + thickness);

        if(faraway)
        {
            EE_tois(i) = toi;
            return;
        }

        bool hit = distance::edge_edge_ccd(
            EP0, EP1, EP2, EP3, dEP0, dEP1, dEP2, dEP3, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        EE_tois(i) = toi;
    }
}  // namespace

REGISTER_SIM_SYSTEM(InfoStacklessBVHSimplexTrajectoryFilter);

void InfoStacklessBVHSimplexTrajectoryFilter::do_build(BuildInfo&)
{
    auto& config = world().scene().config();
    auto  method = config.find<std::string>("collision_detection/method");
    if(method->view()[0] != "info_stackless_bvh")
    {
        throw SimSystemException("Info stackless BVH unused");
    }

    m_impl.query_counts.resize(4);
    m_impl.selected_counts.resize(4);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

cuda_tool::CBufferView<Vector2i> InfoStacklessBVHSimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_impl.candidate_AllP_AllT_pairs.view();
}

cuda_tool::CBufferView<Vector2i> InfoStacklessBVHSimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_impl.candidate_AllE_AllE_pairs.view();
}

cuda_tool::CBufferView<Float> InfoStacklessBVHSimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    return m_impl.tois.view(pp_size + pe_size, pt_size);
}

cuda_tool::CBufferView<Float> InfoStacklessBVHSimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_impl.candidate_AllE_AllE_pairs.size();
    return m_impl.tois.view(pp_size + pe_size + pt_size, ee_size);
}

void InfoStacklessBVHSimplexTrajectoryFilter::Impl::detect(DetectInfo& info)
{
    auto alpha                = info.alpha();
    auto Ps                   = info.positions();
    auto dxs                  = info.displacements();
    auto codimVs              = info.codim_vertices();
    auto Vs                   = info.surf_vertices();
    auto Es                   = info.surf_edges();
    auto Fs                   = info.surf_triangles();
    auto v2bs                 = info.v2b();
    auto body_self_collisions = info.body_self_collision();
    auto contact_element_ids  = info.contact_element_ids();
    auto cmts                 = info.contact_mask_tabular();

    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());
    point_bids.resize(Vs.size());
    triangle_bids.resize(Fs.size());
    edge_bids.resize(Es.size());
    point_cids.resize(Vs.size());
    triangle_cids.resize(Fs.size());
    edge_cids.resize(Es.size());

    // build AABBs for codim vertices
    if(codimVs.size() > 0)
    {
        codim_point_aabbs.resize(codimVs.size());
        codim_point_bids.resize(codimVs.size());
        codim_point_cids.resize(codimVs.size());

        int  n = static_cast<int>(codimVs.size());
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k1_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            codimVs,
            Ps,
            dxs,
            codim_point_aabbs.view(),
            v2bs,
            contact_element_ids,
            codim_point_bids.view(),
            codim_point_cids.view(),
            info.thicknesses(),
            info.d_hats(),
            alpha,
            n);
    }

    // build AABBs for surf vertices (including codim vertices)
    {
        int n = static_cast<int>(Vs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k2_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Vs,
                dxs,
                Ps,
                point_aabbs.view(),
                v2bs,
                contact_element_ids,
                point_bids.view(),
                point_cids.view(),
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
        }
    }

    // build AABBs for edges
    {
        int n = static_cast<int>(Es.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k3_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Es,
                Ps,
                edge_aabbs.view(),
                v2bs,
                contact_element_ids,
                edge_bids.view(),
                edge_cids.view(),
                dxs,
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
        }
    }

    // build AABBs for triangles
    {
        int n = static_cast<int>(Fs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_detect_k4_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                Fs,
                Ps,
                triangle_aabbs.view(),
                v2bs,
                contact_element_ids,
                triangle_bids.view(),
                triangle_cids.view(),
                dxs,
                info.thicknesses(),
                info.d_hats(),
                alpha,
                n);
        }
    }

    lbvh_E.build(edge_aabbs, edge_bids, edge_cids);
    lbvh_T.build(triangle_aabbs, triangle_bids, triangle_cids);

    auto node_pred = InfoStacklessBVHSimplexTrajectoryFilter_detect_node_pred{
        body_self_collisions, cmts.viewer()};
    auto allp_codimp_pred = InfoStacklessBVHSimplexTrajectoryFilter_detect_AllP_CodimP_pred{
        Vs,
        codimVs,
        Ps,
        dxs,
        info.thicknesses(),
        info.dimensions(),
        info.contact_element_ids(),
        info.contact_mask_tabular().viewer(),
        info.subscene_element_ids(),
        info.subscene_mask_tabular().viewer(),
        info.body_self_collision(),
        info.d_hats(),
        alpha};
    auto codimp_alle_pred = InfoStacklessBVHSimplexTrajectoryFilter_detect_CodimP_AllE_pred{
        codimVs,
        Es,
        Ps,
        dxs,
        info.thicknesses(),
        info.contact_element_ids(),
        info.contact_mask_tabular().viewer(),
        info.subscene_element_ids(),
        info.subscene_mask_tabular().viewer(),
        info.body_self_collision(),
        info.d_hats(),
        alpha};
    auto alle_alle_pred = InfoStacklessBVHSimplexTrajectoryFilter_detect_AllE_AllE_pred{
        Es,
        Ps,
        dxs,
        info.thicknesses(),
        info.contact_element_ids(),
        info.contact_mask_tabular().viewer(),
        info.subscene_element_ids(),
        info.subscene_mask_tabular().viewer(),
        info.body_self_collision(),
        info.d_hats(),
        alpha};
    auto allp_allt_pred = InfoStacklessBVHSimplexTrajectoryFilter_detect_AllP_AllT_pred{
        Vs,
        Fs,
        Ps,
        dxs,
        info.thicknesses(),
        info.contact_element_ids(),
        info.contact_mask_tabular().viewer(),
        info.subscene_element_ids(),
        info.subscene_mask_tabular().viewer(),
        info.body_self_collision(),
        info.d_hats(),
        alpha};

    auto launch_allp_codimp = [&](bool rebuild_query)
    {
        lbvh_CodimP.launch_query(point_aabbs,
                                 point_bids,
                                 point_cids,
                                 cmts,
                                 node_pred,
                                 allp_codimp_pred,
                                 candidate_AllP_CodimP_pairs,
                                 rebuild_query);
    };
    auto launch_codimp_alle = [&](bool rebuild_query)
    {
        lbvh_E.launch_query(codim_point_aabbs,
                            codim_point_bids,
                            codim_point_cids,
                            cmts,
                            node_pred,
                            codimp_alle_pred,
                            candidate_CodimP_AllE_pairs,
                            rebuild_query);
    };
    auto launch_alle_alle = [&]
    {
        lbvh_E.launch_detect(cmts, node_pred, alle_alle_pred, candidate_AllE_AllE_pairs);
    };
    auto launch_allp_allt = [&](bool rebuild_query)
    {
        lbvh_T.launch_query(
            point_aabbs, point_bids, point_cids, cmts, node_pred, allp_allt_pred, candidate_AllP_AllT_pairs, rebuild_query);
    };

    if(codimVs.size() > 0)
    {
        lbvh_CodimP.build(codim_point_aabbs, codim_point_bids, codim_point_cids);
        launch_allp_codimp(true);
    }
    else
    {
        candidate_AllP_CodimP_pairs.m_cpNum.fill(0);
    }
    launch_codimp_alle(true);
    launch_alle_alle();
    launch_allp_allt(true);

    InfoStacklessBVHSimplexTrajectoryFilter_collect_query_counts_kernel<<<1, 1>>>(
        candidate_AllP_CodimP_pairs.m_cpNum.cview(),
        candidate_CodimP_AllE_pairs.m_cpNum.cview(),
        candidate_AllE_AllE_pairs.m_cpNum.cview(),
        candidate_AllP_AllT_pairs.m_cpNum.cview(),
        query_counts.view());

    std::array<IndexT, 4> host_counts{};
    query_counts.copy_to(host_counts.data());

    if(lbvh_CodimP.prepare_query_result(candidate_AllP_CodimP_pairs, host_counts[0]))
        launch_allp_codimp(false);
    if(lbvh_E.prepare_query_result(candidate_CodimP_AllE_pairs, host_counts[1]))
        launch_codimp_alle(false);
    if(lbvh_E.prepare_query_result(candidate_AllE_AllE_pairs, host_counts[2]))
        launch_alle_alle();
    if(lbvh_T.prepare_query_result(candidate_AllP_AllT_pairs, host_counts[3]))
        launch_allp_allt(false);
}

void InfoStacklessBVHSimplexTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    using namespace cuda_tool;

    auto positions = info.positions();

    SizeT N_PCoimP  = candidate_AllP_CodimP_pairs.size();
    SizeT N_CodimPE = candidate_CodimP_AllE_pairs.size();
    SizeT N_PTs     = candidate_AllP_AllT_pairs.size();
    SizeT N_EEs     = candidate_AllE_AllE_pairs.size();

    temp_PPs.resize_discard(N_PCoimP + N_CodimPE + N_PTs + N_EEs);
    temp_PEs.resize_discard(N_CodimPE + N_PTs + N_EEs);

    temp_PTs.resize_discard(N_PTs);
    temp_EEs.resize_discard(N_EEs);

    SizeT temp_PP_offset = 0;
    SizeT temp_PE_offset = 0;

    // AllP and CodimP
    if(N_PCoimP > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PCoimP);

        int n = static_cast<int>(candidate_AllP_CodimP_pairs.size());
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k1_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            positions,
            candidate_AllP_CodimP_pairs.view(),
            info.surf_vertices(),
            info.codim_vertices(),
            info.thicknesses(),
            PP_view,
            info.d_hats(),
            n);

        temp_PP_offset += N_PCoimP;
    }
    // CodimP and AllE
    if(N_CodimPE > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_CodimPE);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_CodimPE);

        int n = static_cast<int>(candidate_CodimP_AllE_pairs.size());
        auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k2_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            positions,
            candidate_CodimP_AllE_pairs.view(),
            info.codim_vertices(),
            info.surf_edges(),
            info.thicknesses(),
            PP_view,
            PE_view,
            info.d_hats(),
            n);

        temp_PP_offset += N_CodimPE;
        temp_PE_offset += N_CodimPE;
    }

    // AllP and AllT
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PTs);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_PTs);

        int n = static_cast<int>(candidate_AllP_AllT_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k3_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                positions,
                candidate_AllP_AllT_pairs.view(),
                info.surf_vertices(),
                info.surf_triangles(),
                info.thicknesses(),
                PP_view,
                PE_view,
                temp_PTs.view(),
                info.d_hats(),
                n);
        }

        temp_PP_offset += N_PTs;
        temp_PE_offset += N_PTs;
    }
    // AllE and AllE
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_EEs);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_EEs);

        int n = static_cast<int>(candidate_AllE_AllE_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_active_k4_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                positions,
                info.rest_positions(),
                candidate_AllE_AllE_pairs.view(),
                info.surf_edges(),
                info.thicknesses(),
                PP_view,
                PE_view,
                temp_EEs.view(),
                info.d_hats(),
                n);
        }

        temp_PP_offset += N_EEs;
        temp_PE_offset += N_EEs;
    }

    UIPC_ASSERT(temp_PP_offset == temp_PPs.size(), "size mismatch");
    UIPC_ASSERT(temp_PE_offset == temp_PEs.size(), "size mismatch");

    {
        PPs.resize_discard(temp_PPs.size());
        PEs.resize_discard(temp_PEs.size());
        PTs.resize_discard(temp_PTs.size());
        EEs.resize_discard(temp_EEs.size());

        DeviceSelect().If(temp_PPs.data(),
                          PPs.data(),
                          selected_counts.data(),
                          temp_PPs.size(),
                          InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PP_pred{});

        DeviceSelect().If(temp_PEs.data(),
                          PEs.data(),
                          selected_counts.data() + 1,
                          temp_PEs.size(),
                          InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PE_pred{});

        DeviceSelect().If(temp_PTs.data(),
                          PTs.data(),
                          selected_counts.data() + 2,
                          temp_PTs.size(),
                          InfoStacklessBVHSimplexTrajectoryFilter_filter_active_PT_pred{});

        DeviceSelect().If(temp_EEs.data(),
                          EEs.data(),
                          selected_counts.data() + 3,
                          temp_EEs.size(),
                          InfoStacklessBVHSimplexTrajectoryFilter_filter_active_EE_pred{});

        std::array<IndexT, 4> host_counts{};
        selected_counts.copy_to(host_counts.data());

        IndexT PP_count = host_counts[0];
        IndexT PE_count = host_counts[1];
        IndexT PT_count = host_counts[2];
        IndexT EE_count = host_counts[3];

        PPs.resize_discard(PP_count);
        PEs.resize_discard(PE_count);
        PTs.resize_discard(PT_count);
        EEs.resize_discard(EE_count);
    }

    info.PPs(PPs);
    info.PEs(PEs);
    info.PTs(PTs);
    info.EEs(EEs);

    if constexpr(PrintDebugInfo)
    {
        std::vector<Vector2i> PPs_host;
        std::vector<Float>    PP_thicknesses_host;

        std::vector<Vector3i> PEs_host;
        std::vector<Float>    PE_thicknesses_host;

        std::vector<Vector4i> PTs_host;
        std::vector<Float>    PT_thicknesses_host;

        std::vector<Vector4i> EEs_host;
        std::vector<Float>    EE_thicknesses_host;

        PPs.copy_to(PPs_host);
        PEs.copy_to(PEs_host);
        PTs.copy_to(PTs_host);
        EEs.copy_to(EEs_host);

        std::cout << "filter result:" << std::endl;

        for(auto&& [PP, thickness] : zip(PPs_host, PP_thicknesses_host))
        {
            std::cout << "PP: " << PP.transpose() << " thickness: " << thickness << "\n";
        }

        for(auto&& [PE, thickness] : zip(PEs_host, PE_thicknesses_host))
        {
            std::cout << "PE: " << PE.transpose() << " thickness: " << thickness << "\n";
        }

        for(auto&& [PT, thickness] : zip(PTs_host, PT_thicknesses_host))
        {
            std::cout << "PT: " << PT.transpose() << " thickness: " << thickness << "\n";
        }

        for(auto&& [EE, thickness] : zip(EEs_host, EE_thicknesses_host))
        {
            std::cout << "EE: " << EE.transpose() << " thickness: " << thickness << "\n";
        }

        std::cout << std::flush;
    }
}

void InfoStacklessBVHSimplexTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace cuda_tool;

    auto toi_size =
        candidate_AllP_CodimP_pairs.size() + candidate_CodimP_AllE_pairs.size()
        + candidate_AllP_AllT_pairs.size() + candidate_AllE_AllE_pairs.size();

    tois.resize_discard(toi_size);

    auto offset  = 0;
    auto PP_tois = tois.view(offset, candidate_AllP_CodimP_pairs.size());
    offset += candidate_AllP_CodimP_pairs.size();
    auto PE_tois = tois.view(offset, candidate_CodimP_AllE_pairs.size());
    offset += candidate_CodimP_AllE_pairs.size();
    auto PT_tois = tois.view(offset, candidate_AllP_AllT_pairs.size());
    offset += candidate_AllP_AllT_pairs.size();
    auto EE_tois = tois.view(offset, candidate_AllE_AllE_pairs.size());
    offset += candidate_AllE_AllE_pairs.size();

    UIPC_ASSERT(offset == toi_size, "size mismatch");

    // AllP and CodimP
    {
        int n = static_cast<int>(candidate_AllP_CodimP_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k1_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                PP_tois,
                candidate_AllP_CodimP_pairs.view(),
                info.codim_vertices(),
                info.surf_vertices(),
                info.thicknesses(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n);
        }
    }

    // CodimP and AllE
    {
        int n = static_cast<int>(candidate_CodimP_AllE_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k2_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                PE_tois,
                candidate_CodimP_AllE_pairs.view(),
                info.codim_vertices(),
                info.thicknesses(),
                info.surf_edges(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n);
        }
    }

    // AllP and AllT
    {
        int n = static_cast<int>(candidate_AllP_AllT_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k3_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                PT_tois,
                candidate_AllP_AllT_pairs.view(),
                info.surf_vertices(),
                info.surf_triangles(),
                info.thicknesses(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n);
        }
    }

    // AllE and AllE
    {
        int n = static_cast<int>(candidate_AllE_AllE_pairs.size());
        if(n > 0)
        {
            auto k = InfoStacklessBVHSimplexTrajectoryFilter_filter_toi_k4_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                EE_tois,
                candidate_AllE_AllE_pairs.view(),
                info.surf_edges(),
                info.thicknesses(),
                info.positions(),
                info.displacements(),
                info.d_hats(),
                info.toi_safety_margin(),
                info.alpha(),
                n);
        }
    }

    if(tois.size())
    {
        DeviceReduce().Min(tois.data(), info.toi().data(), tois.size());
    }
    else
    {
        info.toi().fill(large_enough_toi);
    }
}
}  // namespace uipc::backend::cuda
