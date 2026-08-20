#include <collision_detection/filters/stackless_bvh_simplex_trajectory_filter.h>
#include <cuda_tool/cuda_tool.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <utils/distance/distance_flagged.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>
#include <utils/simplex_contact_mask_utils.h>
#include <uipc/common/zip.h>
#include <utils/primitive_d_hat.h>
#include <cstdio>

namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo = false;
constexpr bool PrintKernelZeroDistance = false;

namespace
{
    /****************************************************
    *                   Broad Phase
    ****************************************************/

    __global__ void StacklessBVHSimplexTrajectoryFilter_detect_k1_kernel(
        cuda_tool::CBufferView<IndexT>  codimVs,
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::BufferView<AABB>     aabbs,
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
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_detect_k2_kernel(
        cuda_tool::CBufferView<IndexT>  Vs,
        cuda_tool::CBufferView<Vector3> dxs,
        cuda_tool::CBufferView<Vector3> Ps,
        cuda_tool::BufferView<AABB>     aabbs,
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
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_detect_k3_kernel(
        cuda_tool::CBufferView<Vector2i> Es,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::BufferView<AABB>      aabbs,
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

        Float thickness =
            edge_thickness(thicknesses(eI[0]), thicknesses(eI[1]));
        Float d_hat_expansion =
            edge_dcd_expansion(d_hats(eI[0]), d_hats(eI[1]));

        const auto& pos0   = Ps(eI[0]);
        const auto& pos1   = Ps(eI[1]);
        Vector3     pos0_t = pos0 + dxs(eI[0]) * alpha;
        Vector3     pos1_t = pos1 + dxs(eI[1]) * alpha;

        Vector3 max = pos0_t;
        Vector3 min = pos0_t;

        AABB aabb;

        aabb.extend(pos0.cast<float>())
            .extend(pos1.cast<float>())
            .extend(pos0_t.cast<float>())
            .extend(pos1_t.cast<float>());

        float expand = d_hat_expansion + thickness;

        aabb.min().array() -= expand;
        aabb.max().array() += expand;
        aabbs(i) = aabb;
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_detect_k4_kernel(
        cuda_tool::CBufferView<Vector3i> Fs,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::BufferView<AABB>      aabbs,
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

        Float thickness = triangle_thickness(thicknesses(fI[0]),
                                             thicknesses(fI[1]),
                                             thicknesses(fI[2]));
        Float d_hat_expansion = triangle_dcd_expansion(
            d_hats(fI[0]), d_hats(fI[1]), d_hats(fI[2]));

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
    }

    struct StacklessBVHSimplexTrajectoryFilter_detect_AllP_CodimP_pred
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
        cuda_tool::CBufferView<IndexT>  v2b;
        cuda_tool::CBufferView<IndexT>  body_self_collision;
        cuda_tool::CBufferView<Float>   d_hats;
        Float                           alpha;

        __device__ bool operator()(IndexT i, IndexT j) const
        {
            const auto& V      = Vs(i);
            const auto& codimV = codimVs(j);

            Vector2i cids = {contact_element_ids(V), contact_element_ids(codimV)};
            Vector2i scids = {subscene_element_ids(V), subscene_element_ids(codimV)};

            // discard if the contact is disabled
            if(!allow_PP_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PP_contact(contact_mask_tabular, cids))
                return false;

            bool V_is_codim = dimensions(V) <= 2;  // codim 0D vert and vert from codim 1D edge

            if(V_is_codim && V >= codimV)  // avoid duplicate CodimP-CodimP pairs
                return false;

            auto body_i = v2b(V);
            auto body_j = v2b(codimV);
            // skip self-collision for the same body if self collision off
            if(body_i == body_j && !body_self_collision(body_i))
                return false;


            Vector3 P0  = Ps(V);
            Vector3 dP0 = alpha * dxs(V);

            Vector3 P1  = Ps(codimV);
            Vector3 dP1 = alpha * dxs(codimV);

            Float thickness = PP_thickness(thicknesses(V), thicknesses(codimV));
            Float d_hat = PP_d_hat(d_hats(V), d_hats(codimV));

            Float expand = d_hat + thickness;

            if(!distance::point_point_ccd_broadphase(P0, P1, dP0, dP1, expand))
                return false;

            return true;
        }
    };

    struct StacklessBVHSimplexTrajectoryFilter_detect_CodimP_AllE_pred
    {
        cuda_tool::CBufferView<IndexT>  codimVs;
        cuda_tool::CBufferView<Vector2i> Es;
        cuda_tool::CBufferView<Vector3> Ps;
        cuda_tool::CBufferView<Vector3> dxs;
        cuda_tool::CBufferView<Float>   thicknesses;
        cuda_tool::CBufferView<IndexT>  contact_element_ids;
        cuda_tool::CDense2D<IndexT>     contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>  subscene_element_ids;
        cuda_tool::CDense2D<IndexT>     subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>  v2b;
        cuda_tool::CBufferView<IndexT>  body_self_collision;
        cuda_tool::CBufferView<Float>   d_hats;
        Float                           alpha;

        __device__ bool operator()(IndexT i, IndexT j) const
        {
            const auto& codimV = codimVs(i);
            const auto& E      = Es(j);

            Vector3i cids = {contact_element_ids(codimV),
                             contact_element_ids(E[0]),
                             contact_element_ids(E[1])};

            Vector3i scids = {subscene_element_ids(codimV),
                              subscene_element_ids(E[0]),
                              subscene_element_ids(E[1])};

            // discard if the contact is disabled
            if(!allow_PE_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PE_contact(contact_mask_tabular, cids))
                return false;

            // discard if the vertex is on the edge
            if(E[0] == codimV || E[1] == codimV)
                return false;

            auto body_i = v2b(codimV);
            auto body_j = v2b(E[0]);
            // skip self-collision for the same body if self collision off
            if(body_i == body_j && !body_self_collision(body_i))
                return false;

            Vector3 E0  = Ps(E[0]);
            Vector3 E1  = Ps(E[1]);
            Vector3 dE0 = alpha * dxs(E[0]);
            Vector3 dE1 = alpha * dxs(E[1]);

            Vector3 P  = Ps(codimV);
            Vector3 dP = alpha * dxs(codimV);

            Float thickness = PE_thickness(thicknesses(codimV),
                                           thicknesses(E[0]),
                                           thicknesses(E[1]));
            Float d_hat = PE_d_hat(d_hats(codimV), d_hats(E[0]), d_hats(E[1]));

            Float expand = d_hat + thickness;

            if(!distance::point_edge_ccd_broadphase(P, E0, E1, dP, dE0, dE1, expand))
                return false;

            return true;
        }
    };

    struct StacklessBVHSimplexTrajectoryFilter_detect_AllE_AllE_pred
    {
        cuda_tool::CBufferView<Vector2i> Es;
        cuda_tool::CBufferView<Vector3>  Ps;
        cuda_tool::CBufferView<Vector3>  dxs;
        cuda_tool::CBufferView<Float>    thicknesses;
        cuda_tool::CBufferView<IndexT>   contact_element_ids;
        cuda_tool::CDense2D<IndexT>      contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>   subscene_element_ids;
        cuda_tool::CDense2D<IndexT>      subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>   v2b;
        cuda_tool::CBufferView<IndexT>   body_self_collision;
        cuda_tool::CBufferView<Float>    d_hats;
        Float                            alpha;

        __device__ bool operator()(IndexT i, IndexT j) const
        {
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

            // discard if the contact is disabled
            if(!allow_EE_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_EE_contact(contact_mask_tabular, cids))
                return false;

            // discard if the edges share same vertex
            if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                return false;

            auto body_i = v2b(E0[0]);
            auto body_j = v2b(E1[0]);
            if(body_i == body_j && !body_self_collision(body_i))
                return false;  // skip self-collision for the same body


            Vector3 E0_0  = Ps(E0[0]);
            Vector3 E0_1  = Ps(E0[1]);
            Vector3 dE0_0 = alpha * dxs(E0[0]);
            Vector3 dE0_1 = alpha * dxs(E0[1]);

            Vector3 E1_0  = Ps(E1[0]);
            Vector3 E1_1  = Ps(E1[1]);
            Vector3 dE1_0 = alpha * dxs(E1[0]);
            Vector3 dE1_1 = alpha * dxs(E1[1]);

            Float thickness = EE_thickness(thicknesses(E0[0]),
                                           thicknesses(E0[1]),
                                           thicknesses(E1[0]),
                                           thicknesses(E1[1]));

            Float d_hat =
                EE_d_hat(d_hats(E0[0]), d_hats(E0[1]), d_hats(E1[0]), d_hats(E1[1]));

            Float expand = d_hat + thickness;

            if(!distance::edge_edge_ccd_broadphase(
                   E0_0, E0_1, E1_0, E1_1, dE0_0, dE0_1, dE1_0, dE1_1, expand))
                return false;

            return true;
        }
    };

    struct StacklessBVHSimplexTrajectoryFilter_detect_AllP_AllT_pred
    {
        cuda_tool::CBufferView<IndexT>  Vs;
        cuda_tool::CBufferView<Vector3i> Fs;
        cuda_tool::CBufferView<Vector3> Ps;
        cuda_tool::CBufferView<Vector3> dxs;
        cuda_tool::CBufferView<Float>   thicknesses;
        cuda_tool::CBufferView<IndexT>  contact_element_ids;
        cuda_tool::CDense2D<IndexT>     contact_mask_tabular;
        cuda_tool::CBufferView<IndexT>  subscene_element_ids;
        cuda_tool::CDense2D<IndexT>     subscene_mask_tabular;
        cuda_tool::CBufferView<IndexT>  v2b;
        cuda_tool::CBufferView<IndexT>  body_self_collision;
        cuda_tool::CBufferView<Float>   d_hats;
        Float                           alpha;

        __device__ bool operator()(IndexT i, IndexT j) const
        {
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

            // discard if the contact is disabled
            if(!allow_PT_contact(subscene_mask_tabular, scids))
                return false;
            if(!allow_PT_contact(contact_mask_tabular, cids))
                return false;

            // discard if the point is on the triangle
            if(F[0] == V || F[1] == V || F[2] == V)
                return false;

            auto body_i = v2b(V);
            auto body_j = v2b(F[0]);
            // skip self-collision for the same body if self collision off
            if(body_i == body_j && !body_self_collision(body_i))
                return false;


            Vector3 P  = Ps(V);
            Vector3 dP = alpha * dxs(V);

            Vector3 F0 = Ps(F[0]);
            Vector3 F1 = Ps(F[1]);
            Vector3 F2 = Ps(F[2]);

            Vector3 dF0 = alpha * dxs(F[0]);
            Vector3 dF1 = alpha * dxs(F[1]);
            Vector3 dF2 = alpha * dxs(F[2]);

            Float thickness = PT_thickness(thicknesses(V),
                                           thicknesses(F[0]),
                                           thicknesses(F[1]),
                                           thicknesses(F[2]));

            Float d_hat =
                PT_d_hat(d_hats(V), d_hats(F[0]), d_hats(F[1]), d_hats(F[2]));

            Float expand = d_hat + thickness;

            if(!distance::point_triangle_ccd_broadphase(P, F0, F1, F2, dP, dF0, dF1, dF2, expand))
                return false;

            return true;
        }
    };

    /****************************************************
    *                   Filter Active
    ****************************************************/

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_active_k1_kernel(
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
        // default invalid
        auto& PP = temp_PPs(i);
        PP.setConstant(-1);

        Vector2i indices = PCodimP_pairs(i);

        IndexT P0 = surf_vertices(indices(0));
        IndexT P1 = codim_vertices(indices(1));


        const auto& V0 = positions(P0);
        const auto& V1 = positions(P1);

        Float thickness = PP_thickness(thicknesses(P0), thicknesses(P1));
        Float d_hat = PP_d_hat(d_hats(P0), d_hats(P1));

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_point_distance2(V0, V1, D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf("[SBVH][PP][low-dist] i=%d P=(%d,%d) D=%e range=(%e,%e) "
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
            return;  // early return

        PP = {P0, P1};
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_active_k2_kernel(
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

        Float thickness = PE_thickness(
            thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));

        Float d_hat = PE_d_hat(d_hats(V), d_hats(E(0)), d_hats(E(1)));


        Vector3i flag =
            distance::point_edge_distance_flag(Ps[0], Ps[1], Ps[2]);

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_edge_distance2(flag, Ps[0], Ps[1], Ps[2], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf("[SBVH][PE][low-dist] i=%d V-E=(%d,%d,%d) flag=(%d,%d,%d) "
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
            return;  // early return

        Vector3i offsets;
        auto dim = distance::degenerate_point_edge(flag, offsets);

        switch(dim)
        {
            case 2:  // PP
            {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                PP        = {V0, V1};
            }
            break;
            case 3:  // PE
            {
                PE = vIs;
            }
            break;
            default: {
                UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
            }
            break;
        }
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_active_k3_kernel(
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
        Vector3  Ps[] = {positions(vIs(0)),
                         positions(vIs(1)),
                         positions(vIs(2)),
                         positions(vIs(3))};

        Float thickness = PT_thickness(thicknesses(V),
                                       thicknesses(F(0)),
                                       thicknesses(F(1)),
                                       thicknesses(F(2)));

        Float d_hat =
            PT_d_hat(d_hats(V), d_hats(F(0)), d_hats(F(1)), d_hats(F(2)));

        Vector4i flag =
            distance::point_triangle_distance_flag(Ps[0], Ps[1], Ps[2], Ps[3]);

        Vector2 range = D_range(thickness, d_hat);

        Float D;
        distance::point_triangle_distance2(flag, Ps[0], Ps[1], Ps[2], Ps[3], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf("[SBVH][PT][low-dist] i=%d V-F=(%d,%d,%d,%d) "
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
            return;  // early return

        Vector4i offsets;
        auto dim = distance::degenerate_point_triangle(flag, offsets);

        switch(dim)
        {
            case 2:  // PP
            {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                PP        = {V0, V1};
            }
            break;
            case 3:  // PE
            {
                IndexT V0 = vIs(offsets(0));
                IndexT V1 = vIs(offsets(1));
                IndexT V2 = vIs(offsets(2));
                PE        = {V0, V1, V2};
            }
            break;
            case 4:  // PT
            {
                PT = vIs;
            }
            break;
            default: {
                UIPC_KERNEL_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
            }
            break;
        }
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_active_k4_kernel(
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
        Vector3  Ps[] = {positions(vIs(0)),
                         positions(vIs(1)),
                         positions(vIs(2)),
                         positions(vIs(3))};

        Float thickness = EE_thickness(thicknesses(E0(0)),
                                       thicknesses(E0(1)),
                                       thicknesses(E1(0)),
                                       thicknesses(E1(1)));

        Float d_hat = EE_d_hat(
            d_hats(E0(0)), d_hats(E0(1)), d_hats(E1(0)), d_hats(E1(1)));

        Vector2 range = D_range(thickness, d_hat);

        Vector4i flag =
            distance::edge_edge_distance_flag(Ps[0], Ps[1], Ps[2], Ps[3]);

        Float D;
        distance::edge_edge_distance2(flag, Ps[0], Ps[1], Ps[2], Ps[3], D);

        if constexpr(PrintKernelZeroDistance)
        {
            if(D <= range.x())
            {
                printf("[SBVH][EE][low-dist] i=%d E-E=(%d,%d,%d,%d) "
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
        // Corner case: exact/near-zero EE distance may appear for degenerate or
        // intersecting edge-edge candidates. Treat it as an active EE pair instead
        // of hard-aborting in the trajectory filter stage.
        if(D <= range.x())
        {
            EE = vIs;
            return;
        }
        if(!is_active_D(range, D))
            return;  // early return

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
        else  // classify to EE/PE/PP
        {
            Vector4i offsets;
            auto dim = distance::degenerate_edge_edge(flag, offsets);

            switch(dim)
            {
                case 2:  // PP
                {
                    IndexT V0 = vIs(offsets(0));
                    IndexT V1 = vIs(offsets(1));
                    PP        = {V0, V1};
                }
                break;
                case 3:  // PE
                {
                    IndexT V0 = vIs(offsets(0));
                    IndexT V1 = vIs(offsets(1));
                    IndexT V2 = vIs(offsets(2));
                    PE        = {V0, V1, V2};
                }
                break;
                case 4:  // EE
                {
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

    struct StacklessBVHSimplexTrajectoryFilter_filter_active_valid_PP_pred
    {
        __host__ __device__ bool operator()(const Vector2i& PP) const
        {
            return PP(0) != -1;
        }
    };

    struct StacklessBVHSimplexTrajectoryFilter_filter_active_valid_PE_pred
    {
        __host__ __device__ bool operator()(const Vector3i& PE) const
        {
            return PE(0) != -1;
        }
    };

    struct StacklessBVHSimplexTrajectoryFilter_filter_active_valid_PT_pred
    {
        __host__ __device__ bool operator()(const Vector4i& PT) const
        {
            return PT(0) != -1;
        }
    };

    struct StacklessBVHSimplexTrajectoryFilter_filter_active_valid_EE_pred
    {
        __host__ __device__ bool operator()(const Vector4i& EE) const
        {
            return EE(0) != -1;
        }
    };

    /****************************************************
    *                   CCD TOI
    ****************************************************/

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_toi_k1_kernel(
        cuda_tool::BufferView<Float>     PP_tois,
        cuda_tool::CBufferView<Vector2i> PCodimP_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  positions,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        Float                            eta,
        SizeT                            max_iter,
        Float                            large_enough_toi,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto   indices = PCodimP_pairs(i);
        IndexT V0      = surf_vertices(indices(0));
        IndexT V1      = codim_vertices(indices(1));

        Float thickness = PP_thickness(thicknesses(V0), thicknesses(V1));
        Float d_hat = PP_d_hat(d_hats(V0), d_hats(V1));

        Vector3 VP0  = positions(V0);
        Vector3 VP1  = positions(V1);
        Vector3 dVP0 = alpha * dxs(V0);
        Vector3 dVP1 = alpha * dxs(V1);

        Float toi = large_enough_toi;

        bool faraway = !distance::point_point_ccd_broadphase(
            VP0, VP1, dVP0, dVP1, d_hat + thickness);

        if(faraway)
        {
            PP_tois(i) = toi;
            return;
        }

        bool hit = distance::point_point_ccd(
            VP0, VP1, dVP0, dVP1, eta, thickness, max_iter, toi);

        if(!hit)
            toi = large_enough_toi;

        PP_tois(i) = toi;
    }

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_toi_k2_kernel(
        cuda_tool::BufferView<Float>     PE_tois,
        cuda_tool::CBufferView<Vector2i> CodimP_AllE_pairs,
        cuda_tool::CBufferView<IndexT>   codim_vertices,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        Float                            eta,
        SizeT                            max_iter,
        Float                            large_enough_toi,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto     indices = CodimP_AllE_pairs(i);
        IndexT   V       = codim_vertices(indices(0));
        Vector2i E       = surf_edges(indices(1));

        Float thickness = PE_thickness(
            thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));
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

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_toi_k3_kernel(
        cuda_tool::BufferView<Float>     PT_tois,
        cuda_tool::CBufferView<Vector2i> PT_pairs,
        cuda_tool::CBufferView<IndexT>   surf_vertices,
        cuda_tool::CBufferView<Vector3i> surf_triangles,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        Float                            eta,
        SizeT                            max_iter,
        Float                            large_enough_toi,
        int                              n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        auto     indices = PT_pairs(i);
        IndexT   V       = surf_vertices(indices(0));
        Vector3i F       = surf_triangles(indices(1));

        Float thickness = PT_thickness(thicknesses(V),
                                       thicknesses(F(0)),
                                       thicknesses(F(1)),
                                       thicknesses(F(2)));
        Float d_hat =
            PT_d_hat(d_hats(V), d_hats(F(0)), d_hats(F(1)), d_hats(F(2)));

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

    __global__ void StacklessBVHSimplexTrajectoryFilter_filter_toi_k4_kernel(
        cuda_tool::BufferView<Float>     EE_tois,
        cuda_tool::CBufferView<Vector2i> EE_pairs,
        cuda_tool::CBufferView<Vector2i> surf_edges,
        cuda_tool::CBufferView<Float>    thicknesses,
        cuda_tool::CBufferView<Vector3>  Ps,
        cuda_tool::CBufferView<Vector3>  dxs,
        cuda_tool::CBufferView<Float>    d_hats,
        Float                            alpha,
        Float                            eta,
        SizeT                            max_iter,
        Float                            large_enough_toi,
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

        Float d_hat = EE_d_hat(
            d_hats(E0(0)), d_hats(E0(1)), d_hats(E1(0)), d_hats(E1(1)));


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
            // position
            EP0,
            EP1,
            EP2,
            EP3,
            // displacement
            dEP0,
            dEP1,
            dEP2,
            dEP3,
            d_hat + thickness);

        if(faraway)
        {
            EE_tois(i) = toi;
            return;
        }

        bool hit = distance::edge_edge_ccd(
            // position
            EP0,
            EP1,
            EP2,
            EP3,
            // displacement
            dEP0,
            dEP1,
            dEP2,
            dEP3,
            eta,
            thickness,
            max_iter,
            toi);

        if(!hit)
            toi = large_enough_toi;

        EE_tois(i) = toi;
    }
}  // namespace

REGISTER_SIM_SYSTEM(StacklessBVHSimplexTrajectoryFilter);

void StacklessBVHSimplexTrajectoryFilter::do_build(BuildInfo& info)
{
    auto& config = world().scene().config();
    auto  method = config.find<std::string>("collision_detection/method");
    if(method->view()[0] != "stackless_bvh")
    {
        throw SimSystemException("Stackless BVH unused");
    }
}

void StacklessBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info);
}

void StacklessBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void StacklessBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

cuda_tool::CBufferView<Vector2i> StacklessBVHSimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_impl.candidate_AllP_AllT_pairs.view();
}

cuda_tool::CBufferView<Vector2i> StacklessBVHSimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_impl.candidate_AllE_AllE_pairs.view();
}

cuda_tool::CBufferView<Float> StacklessBVHSimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    return m_impl.tois.view(pp_size + pe_size, pt_size);
}

cuda_tool::CBufferView<Float> StacklessBVHSimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_impl.candidate_AllE_AllE_pairs.size();
    return m_impl.tois.view(pp_size + pe_size + pt_size, ee_size);
}

void StacklessBVHSimplexTrajectoryFilter::Impl::detect(DetectInfo& info)
{
    using namespace cuda_tool;

    auto alpha   = info.alpha();
    auto Ps      = info.positions();
    auto dxs     = info.displacements();
    auto codimVs = info.codim_vertices();
    auto Vs      = info.surf_vertices();
    auto Es      = info.surf_edges();
    auto Fs      = info.surf_triangles();

    //lbvh_E      = {};
    //lbvh_T      = {};
    //lbvh_CodimP = {};

    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());

    // build AABBs for codim vertices
    if(codimVs.size() > 0)
    {
        codim_point_aabbs.resize(codimVs.size());

        auto k1 = StacklessBVHSimplexTrajectoryFilter_detect_k1_kernel;
        k1<<<cuda_tool::best_grid_dim((int)codimVs.size(), k1),
             cuda_tool::best_block_dim(k1),
             0,
             nullptr>>>(codimVs,
                        Ps,
                        dxs,
                        codim_point_aabbs.view(),
                        info.thicknesses(),
                        info.d_hats(),
                        alpha,
                        (int)codimVs.size());
    }

    // build AABBs for surf vertices (including codim vertices)
    if(Vs.size() > 0)
    {
        auto k2 = StacklessBVHSimplexTrajectoryFilter_detect_k2_kernel;
        k2<<<cuda_tool::best_grid_dim((int)Vs.size(), k2),
             cuda_tool::best_block_dim(k2),
             0,
             nullptr>>>(Vs,
                        dxs,
                        Ps,
                        point_aabbs.view(),
                        info.thicknesses(),
                        info.d_hats(),
                        alpha,
                        (int)Vs.size());
    }

    // build AABBs for edges
    if(Es.size() > 0)
    {
        auto k3 = StacklessBVHSimplexTrajectoryFilter_detect_k3_kernel;
        k3<<<cuda_tool::best_grid_dim((int)Es.size(), k3),
             cuda_tool::best_block_dim(k3),
             0,
             nullptr>>>(Es,
                        Ps,
                        edge_aabbs.view(),
                        dxs,
                        info.thicknesses(),
                        info.d_hats(),
                        alpha,
                        (int)Es.size());
    }

    // build AABBs for triangles
    if(Fs.size() > 0)
    {
        auto k4 = StacklessBVHSimplexTrajectoryFilter_detect_k4_kernel;
        k4<<<cuda_tool::best_grid_dim((int)Fs.size(), k4),
             cuda_tool::best_block_dim(k4),
             0,
             nullptr>>>(Fs,
                        Ps,
                        triangle_aabbs.view(),
                        dxs,
                        info.thicknesses(),
                        info.d_hats(),
                        alpha,
                        (int)Fs.size());
    }

    lbvh_E.build(edge_aabbs);
    lbvh_T.build(triangle_aabbs);

    if(codimVs.size() > 0)
    {
        // Use AllP to query CodimP
        {
            lbvh_CodimP.build(codim_point_aabbs);

            lbvh_CodimP.query(
                point_aabbs,  // AllP
                StacklessBVHSimplexTrajectoryFilter_detect_AllP_CodimP_pred{
                    Vs,      // AllP
                    codimVs,  // CodimP
                    Ps,
                    dxs,
                    info.thicknesses(),
                    info.dimensions(),
                    info.contact_element_ids(),
                    info.contact_mask_tabular().viewer(),
                    info.subscene_element_ids(),
                    info.subscene_mask_tabular().viewer(),
                    info.v2b(),
                    info.body_self_collision(),
                    info.d_hats(),
                    alpha},
                candidate_AllP_CodimP_pairs);
        }

        // Use CodimP to query AllE
        {
            lbvh_E.query(codim_point_aabbs,
                         StacklessBVHSimplexTrajectoryFilter_detect_CodimP_AllE_pred{
                             codimVs,
                             Es,
                             Ps,
                             dxs,
                             info.thicknesses(),
                             info.contact_element_ids(),
                             info.contact_mask_tabular().viewer(),
                             info.subscene_element_ids(),
                             info.subscene_mask_tabular().viewer(),
                             info.v2b(),
                             info.body_self_collision(),
                             info.d_hats(),
                             alpha},
                         candidate_CodimP_AllE_pairs);
        }
    }

    // Use AllE to query AllE
    if(Es.size() > 0)
    {
        lbvh_E.detect(StacklessBVHSimplexTrajectoryFilter_detect_AllE_AllE_pred{
                          Es,
                          Ps,
                          dxs,
                          info.thicknesses(),
                          info.contact_element_ids(),
                          info.contact_mask_tabular().viewer(),
                          info.subscene_element_ids(),
                          info.subscene_mask_tabular().viewer(),
                          info.v2b(),
                          info.body_self_collision(),
                          info.d_hats(),
                          alpha},
                      candidate_AllE_AllE_pairs);
    }

    // Use AllP to query AllT
    if(Fs.size() > 0)
    {
        lbvh_T.query(point_aabbs,
                     StacklessBVHSimplexTrajectoryFilter_detect_AllP_AllT_pred{
                         Vs,
                         Fs,
                         Ps,
                         dxs,
                         info.thicknesses(),
                         info.contact_element_ids(),
                         info.contact_mask_tabular().viewer(),
                         info.subscene_element_ids(),
                         info.subscene_mask_tabular().viewer(),
                         info.v2b(),
                         info.body_self_collision(),
                         info.d_hats(),
                         alpha},
                     candidate_AllP_AllT_pairs);
    }
}

void StacklessBVHSimplexTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    using namespace cuda_tool;

    // we will filter-out the active pairs
    auto positions = info.positions();

    SizeT N_PCoimP  = candidate_AllP_CodimP_pairs.size();
    SizeT N_CodimPE = candidate_CodimP_AllE_pairs.size();
    SizeT N_PTs     = candidate_AllP_AllT_pairs.size();
    SizeT N_EEs     = candidate_AllE_AllE_pairs.size();

    // PT, EE, PT, PP can degenerate to PP
    temp_PPs.resize(N_PCoimP + N_CodimPE + N_PTs + N_EEs);
    // PT, EE, PT can degenerate to PE
    temp_PEs.resize(N_CodimPE + N_PTs + N_EEs);

    temp_PTs.resize(N_PTs);
    temp_EEs.resize(N_EEs);

    SizeT temp_PP_offset = 0;
    SizeT temp_PE_offset = 0;

    // AllP and CodimP
    if(N_PCoimP > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PCoimP);

        auto k1 = StacklessBVHSimplexTrajectoryFilter_filter_active_k1_kernel;
        k1<<<cuda_tool::best_grid_dim((int)candidate_AllP_CodimP_pairs.size(), k1),
             cuda_tool::best_block_dim(k1),
             0,
             nullptr>>>(positions,
                        candidate_AllP_CodimP_pairs.viewer(),
                        info.surf_vertices(),
                        info.codim_vertices(),
                        info.thicknesses(),
                        PP_view,
                        info.d_hats(),
                        (int)candidate_AllP_CodimP_pairs.size());

        temp_PP_offset += N_PCoimP;
    }
    // CodimP and AllE
    if(N_CodimPE > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_CodimPE);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_CodimPE);

        auto k2 = StacklessBVHSimplexTrajectoryFilter_filter_active_k2_kernel;
        k2<<<cuda_tool::best_grid_dim((int)candidate_CodimP_AllE_pairs.size(), k2),
             cuda_tool::best_block_dim(k2),
             0,
             nullptr>>>(positions,
                        candidate_CodimP_AllE_pairs.viewer(),
                        info.codim_vertices(),
                        info.surf_edges(),
                        info.thicknesses(),
                        PP_view,
                        PE_view,
                        info.d_hats(),
                        (int)candidate_CodimP_AllE_pairs.size());

        temp_PP_offset += N_CodimPE;
        temp_PE_offset += N_CodimPE;
    }

    // AllP and AllT
    if(N_PTs > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PTs);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_PTs);

        auto k3 = StacklessBVHSimplexTrajectoryFilter_filter_active_k3_kernel;
        k3<<<cuda_tool::best_grid_dim((int)candidate_AllP_AllT_pairs.size(), k3),
             cuda_tool::best_block_dim(k3),
             0,
             nullptr>>>(positions,
                        candidate_AllP_AllT_pairs.viewer(),
                        info.surf_vertices(),
                        info.surf_triangles(),
                        info.thicknesses(),
                        PP_view,
                        PE_view,
                        temp_PTs.view(),
                        info.d_hats(),
                        (int)candidate_AllP_AllT_pairs.size());

        temp_PP_offset += N_PTs;
        temp_PE_offset += N_PTs;
    }
    // AllE and AllE
    if(N_EEs > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_EEs);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_EEs);

        auto k4 = StacklessBVHSimplexTrajectoryFilter_filter_active_k4_kernel;
        k4<<<cuda_tool::best_grid_dim((int)candidate_AllE_AllE_pairs.size(), k4),
             cuda_tool::best_block_dim(k4),
             0,
             nullptr>>>(positions,
                        info.rest_positions(),
                        candidate_AllE_AllE_pairs.viewer(),
                        info.surf_edges(),
                        info.thicknesses(),
                        PP_view,
                        PE_view,
                        temp_EEs.view(),
                        info.d_hats(),
                        (int)candidate_AllE_AllE_pairs.size());

        temp_PP_offset += N_EEs;
        temp_PE_offset += N_EEs;
    }

    UIPC_ASSERT(temp_PP_offset == temp_PPs.size(), "size mismatch");
    UIPC_ASSERT(temp_PE_offset == temp_PEs.size(), "size mismatch");

    {  // select the valid ones
        PPs.resize(temp_PPs.size());
        PEs.resize(temp_PEs.size());
        PTs.resize(temp_PTs.size());
        EEs.resize(temp_EEs.size());

        DeviceSelect().If(temp_PPs.data(),
                          PPs.data(),
                          selected_PP_count.data(),
                          temp_PPs.size(),
                          StacklessBVHSimplexTrajectoryFilter_filter_active_valid_PP_pred{});

        DeviceSelect().If(temp_PEs.data(),
                          PEs.data(),
                          selected_PE_count.data(),
                          temp_PEs.size(),
                          StacklessBVHSimplexTrajectoryFilter_filter_active_valid_PE_pred{});

        DeviceSelect().If(temp_PTs.data(),
                          PTs.data(),
                          selected_PT_count.data(),
                          temp_PTs.size(),
                          StacklessBVHSimplexTrajectoryFilter_filter_active_valid_PT_pred{});

        DeviceSelect().If(temp_EEs.data(),
                          EEs.data(),
                          selected_EE_count.data(),
                          temp_EEs.size(),
                          StacklessBVHSimplexTrajectoryFilter_filter_active_valid_EE_pred{});

        IndexT PP_count = selected_PP_count;
        IndexT PE_count = selected_PE_count;
        IndexT PT_count = selected_PT_count;
        IndexT EE_count = selected_EE_count;

        PPs.resize(PP_count);
        PEs.resize(PE_count);
        PTs.resize(PT_count);
        EEs.resize(EE_count);
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

void StacklessBVHSimplexTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace cuda_tool;

    auto toi_size =
        candidate_AllP_CodimP_pairs.size() + candidate_CodimP_AllE_pairs.size()
        + candidate_AllP_AllT_pairs.size() + candidate_AllE_AllE_pairs.size();

    tois.resize(toi_size);

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


    // TODO: Now hard code the minimum separation coefficient
    // gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
    constexpr Float eta = 0.1;

    // TODO: Now hard code the maximum iteration
    constexpr SizeT max_iter = 1000;

    // large enough toi (>1)
    constexpr Float large_enough_toi = 1.1;

    // AllP and CodimP
    if(candidate_AllP_CodimP_pairs.size() > 0)
    {
        auto k1 = StacklessBVHSimplexTrajectoryFilter_filter_toi_k1_kernel;
        k1<<<cuda_tool::best_grid_dim((int)candidate_AllP_CodimP_pairs.size(), k1),
             cuda_tool::best_block_dim(k1),
             0,
             nullptr>>>(PP_tois,
                        candidate_AllP_CodimP_pairs.viewer(),
                        info.codim_vertices(),
                        info.surf_vertices(),
                        info.thicknesses(),
                        info.positions(),
                        info.displacements(),
                        info.d_hats(),
                        info.alpha(),
                        eta,
                        max_iter,
                        large_enough_toi,
                        (int)candidate_AllP_CodimP_pairs.size());
    }

    // CodimP and AllE
    if(candidate_CodimP_AllE_pairs.size() > 0)
    {
        auto k2 = StacklessBVHSimplexTrajectoryFilter_filter_toi_k2_kernel;
        k2<<<cuda_tool::best_grid_dim((int)candidate_CodimP_AllE_pairs.size(), k2),
             cuda_tool::best_block_dim(k2),
             0,
             nullptr>>>(PE_tois,
                        candidate_CodimP_AllE_pairs.viewer(),
                        info.codim_vertices(),
                        info.thicknesses(),
                        info.surf_edges(),
                        info.positions(),
                        info.displacements(),
                        info.d_hats(),
                        info.alpha(),
                        eta,
                        max_iter,
                        large_enough_toi,
                        (int)candidate_CodimP_AllE_pairs.size());
    }

    // AllP and AllT
    if(candidate_AllP_AllT_pairs.size() > 0)
    {
        auto k3 = StacklessBVHSimplexTrajectoryFilter_filter_toi_k3_kernel;
        k3<<<cuda_tool::best_grid_dim((int)candidate_AllP_AllT_pairs.size(), k3),
             cuda_tool::best_block_dim(k3),
             0,
             nullptr>>>(PT_tois,
                        candidate_AllP_AllT_pairs.viewer(),
                        info.surf_vertices(),
                        info.surf_triangles(),
                        info.thicknesses(),
                        info.positions(),
                        info.displacements(),
                        info.d_hats(),
                        info.alpha(),
                        eta,
                        max_iter,
                        large_enough_toi,
                        (int)candidate_AllP_AllT_pairs.size());
    }

    // AllE and AllE
    if(candidate_AllE_AllE_pairs.size() > 0)
    {
        auto k4 = StacklessBVHSimplexTrajectoryFilter_filter_toi_k4_kernel;
        k4<<<cuda_tool::best_grid_dim((int)candidate_AllE_AllE_pairs.size(), k4),
             cuda_tool::best_block_dim(k4),
             0,
             nullptr>>>(EE_tois,
                        candidate_AllE_AllE_pairs.viewer(),
                        info.surf_edges(),
                        info.thicknesses(),
                        info.positions(),
                        info.displacements(),
                        info.d_hats(),
                        info.alpha(),
                        eta,
                        max_iter,
                        large_enough_toi,
                        (int)candidate_AllE_AllE_pairs.size());
    }

    if(tois.size())
    {
        // get min toi
        DeviceReduce().Min(tois.data(), info.toi().data(), tois.size());
    }
    else
    {
        info.toi().fill(large_enough_toi);
    }
}
}  // namespace uipc::backend::cuda
