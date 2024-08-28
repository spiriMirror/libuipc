#include <collision_detection/filters/lbvh_simplex_trajectory_filter.h>
#include <muda/cub/device/device_select.h>
#include <muda/ext/eigen/log_proxy.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <utils/distance/distance_type.h>
#include <utils/distance.h>
#include <utils/codim_thickness.h>
#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo = false;

REGISTER_SIM_SYSTEM(LBVHSimplexTrajectoryFilter);

void LBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info);
}

void LBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void LBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

void LBVHSimplexTrajectoryFilter::Impl::detect(DetectInfo& info)
{
    using namespace muda;

    auto alpha   = info.alpha();
    auto d_hat   = info.d_hat();
    auto Ps      = info.positions();
    auto dxs     = info.displacements();
    auto codimVs = info.codim_vertices();
    auto Vs      = info.surf_vertices();
    auto Es      = info.surf_edges();
    auto Fs      = info.surf_triangles();

    codim_point_aabbs.resize(codimVs.size());
    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());

    // build AABBs for codim vertices
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(codimVs.size(),
               [codimVs     = codimVs.viewer().name("codimVs"),
                Ps          = Ps.viewer().name("Ps"),
                dxs         = dxs.viewer().name("dxs"),
                aabbs       = codim_point_aabbs.viewer().name("aabbs"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                alpha       = alpha,
                d_hat       = d_hat] __device__(int i) mutable
               {
                   auto vI = codimVs(i);

                   Float thickness = thicknesses(vI);

                   const auto& pos   = Ps(vI);
                   Vector3     pos_t = pos + dxs(vI) * alpha;

                   AABB aabb;
                   aabb.extend(pos).extend(pos_t);

                   Float expand = d_hat + thickness;

                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;
                   aabbs(i) = aabb;
               });


    // build AABBs for surf vertices (including codim vertices)
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(Vs.size(),
               [Vs          = Vs.viewer().name("V"),
                dxs         = dxs.viewer().name("dx"),
                Ps          = Ps.viewer().name("Ps"),
                aabbs       = point_aabbs.viewer().name("aabbs"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                alpha       = alpha,
                d_hat       = d_hat] __device__(int i) mutable
               {
                   auto vI = Vs(i);

                   Float thickness = thicknesses(vI);

                   const auto& pos   = Ps(vI);
                   Vector3     pos_t = pos + dxs(vI) * alpha;

                   AABB aabb;
                   aabb.extend(pos).extend(pos_t);

                   Float expand = d_hat + thickness;

                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;
                   aabbs(i) = aabb;
               });

    // build AABBs for edges
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(Es.size(),
               [Es          = Es.viewer().name("E"),
                Ps          = Ps.viewer().name("Ps"),
                aabbs       = edge_aabbs.viewer().name("aabbs"),
                dxs         = dxs.viewer().name("dx"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                alpha       = alpha,
                d_hat       = d_hat] __device__(int i) mutable
               {
                   auto eI = Es(i);

                   Float thickness =
                       edge_thickness(thicknesses(eI[0]), thicknesses(eI[1]));

                   const auto& pos0   = Ps(eI[0]);
                   const auto& pos1   = Ps(eI[1]);
                   Vector3     pos0_t = pos0 + dxs(eI[0]) * alpha;
                   Vector3     pos1_t = pos1 + dxs(eI[1]) * alpha;

                   Vector3 max = pos0_t;
                   Vector3 min = pos0_t;

                   AABB aabb;

                   aabb.extend(pos0).extend(pos1).extend(pos0_t).extend(pos1_t);

                   Float expand = d_hat + thickness;

                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;
                   aabbs(i) = aabb;
               });

    // build AABBs for triangles
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(Fs.size(),
               [Fs          = Fs.viewer().name("F"),
                Ps          = Ps.viewer().name("Ps"),
                aabbs       = triangle_aabbs.viewer().name("aabbs"),
                dxs         = dxs.viewer().name("dx"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                alpha       = alpha,
                d_hat       = d_hat] __device__(int i) mutable
               {
                   auto fI = Fs(i);

                   Float thickness = triangle_thickness(thicknesses(fI[0]),
                                                        thicknesses(fI[1]),
                                                        thicknesses(fI[2]));

                   const auto& pos0   = Ps(fI[0]);
                   const auto& pos1   = Ps(fI[1]);
                   const auto& pos2   = Ps(fI[2]);
                   Vector3     pos0_t = pos0 + dxs(fI[0]) * alpha;
                   Vector3     pos1_t = pos1 + dxs(fI[1]) * alpha;
                   Vector3     pos2_t = pos2 + dxs(fI[2]) * alpha;

                   AABB aabb;

                   aabb.extend(pos0)
                       .extend(pos1)
                       .extend(pos2)
                       .extend(pos0_t)
                       .extend(pos1_t)
                       .extend(pos2_t);

                   Float expand = d_hat + thickness;

                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;
                   aabbs(i) = aabb;
               });

    // query CodimP and P
    lbvh_PP.build(point_aabbs);
    candidate_PP_pairs = lbvh_PP.query(
        codim_point_aabbs,
        [codimVs     = codimVs.viewer().name("codimVs"),
         Vs          = Vs.viewer().name("Vs"),
         Ps          = Ps.viewer().name("Ps"),
         dxs         = dxs.viewer().name("dxs"),
         thicknesses = info.thicknesses().viewer().name("thicknesses"),
         d_hat       = d_hat,
         alpha       = alpha] __device__(IndexT i, IndexT j)
        {
            const auto& codimV = codimVs(i);
            const auto& V      = Vs(j);

            Vector3 P0  = Ps(codimV);
            Vector3 P1  = Ps(V);
            Vector3 dP0 = alpha * dxs(codimV);
            Vector3 dP1 = alpha * dxs(V);

            Float thickness = PP_thickness(thicknesses(codimV), thicknesses(V));

            Float expand = d_hat + thickness;

            if(!distance::point_point_ccd_broadphase(P0, P1, dP0, dP1, expand))
                return false;

            return true;
        });

    // query PE
    lbvh_PE.build(edge_aabbs);
    candidate_PE_pairs = lbvh_PE.query(
        codim_point_aabbs,
        [codimVs     = codimVs.viewer().name("Vs"),
         Es          = Es.viewer().name("Es"),
         Ps          = Ps.viewer().name("Ps"),
         dxs         = dxs.viewer().name("dxs"),
         thicknesses = info.thicknesses().viewer().name("thicknesses"),
         d_hat       = d_hat,
         alpha       = alpha] __device__(IndexT i, IndexT j)
        {
            const auto& codimV = codimVs(i);
            const auto& E      = Es(j);

            Vector3 E0  = Ps(E[0]);
            Vector3 E1  = Ps(E[1]);
            Vector3 dE0 = alpha * dxs(E[0]);
            Vector3 dE1 = alpha * dxs(E[1]);

            Vector3 P  = Ps(codimV);
            Vector3 dP = alpha * dxs(codimV);

            Float thickness =
                PE_thickness(thicknesses(codimV), thicknesses(E[0]), thicknesses(E[1]));

            Float expand = d_hat + thickness;

            if(!distance::point_edge_ccd_broadphase(P, E0, E1, dP, dE0, dE1, expand))
                return false;

            return true;
        });


    // query PT
    lbvh_PT.build(triangle_aabbs);
    candidate_PT_pairs = lbvh_PT.query(
        point_aabbs,
        [Vs          = Vs.viewer().name("Vs"),
         Fs          = Fs.viewer().name("Fs"),
         Ps          = Ps.viewer().name("Ps"),
         dxs         = dxs.viewer().name("dxs"),
         thicknesses = info.thicknesses().viewer().name("thicknesses"),
         d_hat       = d_hat,
         alpha       = alpha] __device__(IndexT i, IndexT j)
        {
            // discard if the point is on the triangle
            auto V = Vs(i);
            auto F = Fs(j);

            if(F[0] == V || F[1] == V || F[2] == V)
                return false;

            Vector3 P  = Ps(V);
            Vector3 dP = alpha * dxs(V);

            Vector3 F0 = Ps(F[0]);
            Vector3 F1 = Ps(F[1]);
            Vector3 F2 = Ps(F[2]);

            Vector3 dF0 = alpha * dxs(F[0]);
            Vector3 dF1 = alpha * dxs(F[1]);
            Vector3 dF2 = alpha * dxs(F[2]);

            Float thickness = triangle_thickness(
                thicknesses(F[0]), thicknesses(F[1]), thicknesses(F[2]));

            Float expand = d_hat + thickness;

            if(!distance::point_triangle_ccd_broadphase(P, F0, F1, F2, dP, dF0, dF1, dF2, expand))
                return false;

            return true;
        });

    // query EE
    lbvh_EE.build(edge_aabbs);
    candidate_EE_pairs = lbvh_EE.detect(
        [Es          = Es.viewer().name("Es"),
         Ps          = Ps.viewer().name("Ps"),
         dxs         = dxs.viewer().name("dxs"),
         thicknesses = info.thicknesses().viewer().name("thicknesses"),
         d_hat       = d_hat,
         alpha       = alpha] __device__(IndexT i, IndexT j)
        {
            // discard if the edges shared same vertex
            auto Ea = Es(i);
            auto Eb = Es(j);

            if(Ea[0] == Eb[0] || Ea[0] == Eb[1] || Ea[1] == Eb[0] || Ea[1] == Eb[1])
                return false;

            Vector3 Ea0  = Ps(Ea[0]);
            Vector3 Ea1  = Ps(Ea[1]);
            Vector3 dEa0 = alpha * dxs(Ea[0]);
            Vector3 dEa1 = alpha * dxs(Ea[1]);

            Vector3 Eb0  = Ps(Eb[0]);
            Vector3 Eb1  = Ps(Eb[1]);
            Vector3 dEb0 = alpha * dxs(Eb[0]);
            Vector3 dEb1 = alpha * dxs(Eb[1]);

            Float thickness = EE_thickness(thicknesses(Ea[0]),
                                           thicknesses(Ea[1]),
                                           thicknesses(Eb[0]),
                                           thicknesses(Eb[1]));

            Float expand = d_hat + thickness;

            if(!distance::edge_edge_ccd_broadphase(
                   // position
                   Ea0,
                   Ea1,
                   Eb0,
                   Eb1,
                   // displacement
                   dEa0,
                   dEa1,
                   dEb0,
                   dEb1,
                   expand))
                return false;

            return true;
        });
}

void LBVHSimplexTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    using namespace muda;

    // we will filter-out the active pairs

    auto d_hat     = info.d_hat();
    auto positions = info.positions();

    auto get_total_count = [](muda::DeviceBuffer<IndexT>& offsets)
    {
        IndexT total_count = 0;
        offsets.view(offsets.size() - 1).copy_to(&total_count);
        return total_count;
    };

    // PPs
    {
        // +1 for total count
        PP_active_flags.resize(candidate_PP_pairs.size() + 1);
        PP_active_offsets.resize(candidate_PP_pairs.size() + 1);

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PP_pairs.size(),
                   [positions = positions.viewer().name("positions"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    PP_pairs = candidate_PP_pairs.viewer().name("PP_pairs"),
                    actives  = PP_active_flags.viewer().name("actives"),
                    d_hat    = d_hat] __device__(int i) mutable
                   {
                       // clear flag
                       actives(i)       = 0;
                       Vector2i indices = PP_pairs(i);

                       IndexT P0 = surf_vertices(indices(0));
                       IndexT P1 = surf_vertices(indices(1));

                       const auto& V0 = positions(P0);
                       const auto& V1 = positions(P1);

                       Float D;
                       distance::point_point_distance_unclassified(V0, V1, D);

                       Float thickness = PP_thickness(thicknesses(P0), thicknesses(P1));
                       Vector2 range = D_range(thickness, d_hat);

                       if(is_active_D(range, D))
                           actives(i) = 1;  // must use 1, we will scan the active pairs later
                   });

        // scan the active pairs
        DeviceScan().ExclusiveSum(PP_active_flags.data(),
                                  PP_active_offsets.data(),
                                  PP_active_offsets.size());

        IndexT total_count = get_total_count(PP_active_offsets);

        PPs.resize(total_count);

        // copy the active pairs
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PP_pairs.size(),
                   [PP_active_flags = PP_active_flags.viewer().name("PP_active_flags"),
                    PP_active_offsets = PP_active_offsets.viewer().name("PP_active_offsets"),
                    PP_pairs = candidate_PP_pairs.viewer().name("PP_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    PPs = PPs.viewer().name("PPs")] __device__(int i) mutable
                   {
                       if(PP_active_flags(i))
                       {
                           auto     offset  = PP_active_offsets(i);
                           Vector2i surf_vI = PP_pairs(i);

                           IndexT P0 = surf_vertices(surf_vI(0));
                           IndexT P1 = surf_vertices(surf_vI(1));

                           PPs(offset) = {P0, P1};
                       }
                   });
    }
    // PEs
    {
        // +1 for total count
        PE_active_flags.resize(candidate_PE_pairs.size() + 1);
        PE_active_offsets.resize(candidate_PE_pairs.size() + 1);

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PE_pairs.size(),
                   [positions = positions.viewer().name("positions"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    PE_pairs   = candidate_PE_pairs.viewer().name("PE_pairs"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    actives = PE_active_flags.viewer().name("actives"),
                    d_hat   = d_hat] __device__(int i) mutable
                   {
                       // clear flag
                       actives(i) = 0;

                       Vector2i indices = PE_pairs(i);
                       IndexT   P       = surf_vertices(indices(0));
                       Vector2i E       = surf_edges(indices(1));

                       const auto& V  = positions(P);
                       const auto& E0 = positions(E(0));
                       const auto& E1 = positions(E(1));

                       Float D;
                       distance::point_edge_distance_unclassified(V, E0, E1, D);

                       Float thickness = PE_thickness(
                           thicknesses(P), thicknesses(E(0)), thicknesses(E(1)));

                       Vector2 range = D_range(thickness, d_hat);

                       if(is_active_D(range, D))
                           actives(i) = 1;  // must use 1, we will scan the active pairs later
                   });

        // scan the active pairs

        DeviceScan().ExclusiveSum(PE_active_flags.data(),
                                  PE_active_offsets.data(),
                                  PE_active_offsets.size());

        IndexT total_count = get_total_count(PE_active_offsets);

        PEs.resize(total_count);

        // copy the active pairs
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PE_pairs.size(),
                   [PE_active_flags = PE_active_flags.viewer().name("PE_active_flags"),
                    PE_active_offsets = PE_active_offsets.viewer().name("PE_active_offsets"),
                    PE_pairs = candidate_PE_pairs.viewer().name("PE_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    PEs = PEs.viewer().name("PEs")] __device__(int i) mutable
                   {
                       if(PE_active_flags(i))
                       {
                           auto     offset       = PE_active_offsets(i);
                           Vector2i surf_indices = PE_pairs(i);

                           IndexT   P = surf_vertices(surf_indices(0));
                           Vector2i E = surf_edges(surf_indices(1));

                           PEs(offset) = {P, E(0), E(1)};
                       }
                   });
    }
    // PTs
    {
        // +1 for total count
        PT_active_flags.resize(candidate_PT_pairs.size() + 1);
        PT_active_offsets.resize(candidate_PT_pairs.size() + 1);

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PT_pairs.size(),
                   [Ps       = positions.viewer().name("Ps"),
                    PT_pairs = candidate_PT_pairs.viewer().name("PT_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    surf_triangles = info.surf_triangles().viewer().name("surf_triangles"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    actives = PT_active_flags.viewer().name("actives"),
                    d_hat   = d_hat] __device__(int i) mutable
                   {
                       // clear flag
                       actives(i) = 0;

                       Vector2i indices = PT_pairs(i);
                       IndexT   P       = surf_vertices(indices(0));
                       Vector3i T       = surf_triangles(indices(1));

                       const auto& V  = Ps(P);
                       const auto& F0 = Ps(T(0));
                       const auto& F1 = Ps(T(1));
                       const auto& F2 = Ps(T(2));


                       Float D;
                       distance::point_triangle_distance_unclassified(V, F0, F1, F2, D);

                       Float thickness = triangle_thickness(thicknesses(T(0)),
                                                            thicknesses(T(1)),
                                                            thicknesses(T(2)));

                       Vector2 range = D_range(thickness, d_hat);

                       if(is_active_D(range, D))
                           actives(i) = 1;  // must use 1, we will scan the active pairs later
                   });

        // scan the active pairs
        DeviceScan().ExclusiveSum(PT_active_flags.data(),
                                  PT_active_offsets.data(),
                                  PT_active_offsets.size());

        IndexT total_count = get_total_count(PT_active_offsets);

        PTs.resize(total_count);

        // copy the active pairs

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PT_pairs.size(),
                   [PT_active_flags = PT_active_flags.viewer().name("PT_active_flags"),
                    PT_active_offsets = PT_active_offsets.viewer().name("PT_active_offsets"),
                    PT_pairs = candidate_PT_pairs.viewer().name("PT_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    surf_triangles = info.surf_triangles().viewer().name("surf_triangles"),
                    PTs = PTs.viewer().name("PTs")] __device__(int i) mutable
                   {
                       if(PT_active_flags(i))
                       {
                           auto     offset       = PT_active_offsets(i);
                           Vector2i surf_indices = PT_pairs(i);

                           IndexT   P = surf_vertices(surf_indices(0));
                           Vector3i T = surf_triangles(surf_indices(1));

                           PTs(offset) = {P, T(0), T(1), T(2)};
                       }
                   });
    }
    // EEs
    {
        // +1 for total count
        EE_active_flags.resize(candidate_EE_pairs.size() + 1);
        EE_active_offsets.resize(candidate_EE_pairs.size() + 1);

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_EE_pairs.size(),
                   [Ps         = positions.viewer().name("Ps"),
                    EE_pairs   = candidate_EE_pairs.viewer().name("EE_pairs"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    actives = EE_active_flags.viewer().name("actives"),
                    d_hat   = d_hat] __device__(int i) mutable
                   {
                       // clear flag
                       actives(i) = 0;

                       Vector2i indices = EE_pairs(i);
                       Vector2i E0      = surf_edges(indices(0));
                       Vector2i E1      = surf_edges(indices(1));

                       const auto& EP0 = Ps(E0(0));
                       const auto& EP1 = Ps(E0(1));
                       const auto& EP2 = Ps(E1(0));
                       const auto& EP3 = Ps(E1(1));

                       Float D;
                       distance::edge_edge_distance_unclassified(EP0, EP1, EP2, EP3, D);

                       Float thickness = EE_thickness(thicknesses(E0(0)),
                                                      thicknesses(E0(1)),
                                                      thicknesses(E1(0)),
                                                      thicknesses(E1(1)));

                       Vector2 range = D_range(thickness, d_hat);

                       if(is_active_D(range, D))
                           actives(i) = 1;  // must use 1, we will scan the active pairs later
                   });

        // scan the active pairs

        DeviceScan().ExclusiveSum(EE_active_flags.data(),
                                  EE_active_offsets.data(),
                                  EE_active_offsets.size());

        IndexT total_count = get_total_count(EE_active_offsets);

        EEs.resize(total_count);

        // copy the active pairs
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_EE_pairs.size(),
                   [EE_active_flags = EE_active_flags.viewer().name("EE_active_flags"),
                    EE_active_offsets = EE_active_offsets.viewer().name("EE_active_offsets"),
                    EE_pairs   = candidate_EE_pairs.viewer().name("EE_pairs"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    EEs = EEs.viewer().name("EEs")] __device__(int i) mutable
                   {
                       if(EE_active_flags(i))
                       {
                           auto     offset       = EE_active_offsets(i);
                           Vector2i surf_indices = EE_pairs(i);

                           Vector2i E0 = surf_edges(surf_indices(0));
                           Vector2i E1 = surf_edges(surf_indices(1));

                           EEs(offset) = {E0(0), E0(1), E1(0), E1(1)};
                       }
                   });
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

void LBVHSimplexTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace muda;

    auto toi_size = candidate_PP_pairs.size() + candidate_PE_pairs.size()
                    + candidate_PT_pairs.size() + candidate_EE_pairs.size();

    tois.resize(toi_size);

    auto offset  = 0;
    auto PP_tois = tois.view(offset, candidate_PP_pairs.size());
    offset += candidate_PP_pairs.size();
    auto PE_tois = tois.view(offset, candidate_PE_pairs.size());
    offset += candidate_PE_pairs.size();
    auto PT_tois = tois.view(offset, candidate_PT_pairs.size());
    offset += candidate_PT_pairs.size();
    auto EE_tois = tois.view(offset, candidate_EE_pairs.size());


    // TODO: Now hard code the minimum separation coefficient
    // gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
    constexpr Float eta = 0.1;

    // TODO: Now hard code the maximum iteration
    constexpr SizeT max_iter = 1000;

    // large enough toi (>1)
    constexpr Float large_enough_toi = 1.1;

    // PP
    {
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PP_pairs.size(),
                   [PP_tois  = PP_tois.viewer().name("PP_tois"),
                    PP_pairs = candidate_PP_pairs.viewer().name("PP_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    positions = info.positions().viewer().name("Ps"),
                    dxs       = info.displacements().viewer().name("dxs"),
                    alpha     = info.alpha(),
                    d_hat     = info.d_hat(),
                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
                       auto   indices = PP_pairs(i);
                       IndexT V0      = surf_vertices(indices(0));
                       IndexT V1      = surf_vertices(indices(1));

                       Float thickness = PP_thickness(thicknesses(V0), thicknesses(V1));

                       Vector3 VP0  = positions(V0);
                       Vector3 VP1  = positions(V1);
                       Vector3 dVP0 = alpha * dxs(V0);
                       Vector3 dVP1 = alpha * dxs(V1);

                       Float toi = large_enough_toi;

                       bool faraway =
                           !distance::point_point_ccd_broadphase(VP0, VP1, dVP0, dVP1, d_hat);

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
                   });
    }

    // PE
    {
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PE_pairs.size(),
                   [PE_tois  = PE_tois.viewer().name("PE_tois"),
                    PE_pairs = candidate_PE_pairs.viewer().name("PE_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    Ps         = info.positions().viewer().name("Ps"),
                    dxs        = info.displacements().viewer().name("dxs"),
                    alpha      = info.alpha(),
                    d_hat      = info.d_hat(),
                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
                       auto     indices   = PE_pairs(i);
                       IndexT   V         = surf_vertices(indices(0));
                       Vector2i E         = surf_edges(indices(1));
                       Float    thickness = PE_thickness(
                           thicknesses(V), thicknesses(E(0)), thicknesses(E(1)));

                       Vector3 VP  = Ps(V);
                       Vector3 dVP = alpha * dxs(V);

                       Vector3 EP0  = Ps(E[0]);
                       Vector3 EP1  = Ps(E[1]);
                       Vector3 dEP0 = alpha * dxs(E[0]);
                       Vector3 dEP1 = alpha * dxs(E[1]);

                       Float toi = large_enough_toi;

                       bool faraway = !distance::point_edge_ccd_broadphase(
                           VP, EP0, EP1, dVP, dEP0, dEP1, d_hat);

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
                   });
    }

    // PT
    {
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_PT_pairs.size(),
                   [PT_tois  = PT_tois.viewer().name("PT_tois"),
                    PT_pairs = candidate_PT_pairs.viewer().name("PT_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    surf_triangles = info.surf_triangles().viewer().name("surf_triangles"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    Ps  = info.positions().viewer().name("Ps"),
                    dxs = info.displacements().viewer().name("dxs"),

                    alpha = info.alpha(),
                    d_hat = info.d_hat(),
                    eta,

                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
                       auto     indices   = PT_pairs(i);
                       IndexT   V         = surf_vertices(indices(0));
                       Vector3i F         = surf_triangles(indices(1));
                       Float    thickness = PT_thickness(thicknesses(V),
                                                      thicknesses(F(0)),
                                                      thicknesses(F(1)),
                                                      thicknesses(F(2)));

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
                           VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, d_hat);

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
                   });
    }

    // EE
    {
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(candidate_EE_pairs.size(),
                   [EE_tois    = EE_tois.viewer().name("EE_tois"),
                    EE_pairs   = candidate_EE_pairs.viewer().name("EE_pairs"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    Ps    = info.positions().viewer().name("Ps"),
                    dxs   = info.displacements().viewer().name("dxs"),
                    alpha = info.alpha(),
                    d_hat = info.d_hat(),
                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
                       auto     indices   = EE_pairs(i);
                       Vector2i E0        = surf_edges(indices(0));
                       Vector2i E1        = surf_edges(indices(1));
                       Float    thickness = EE_thickness(thicknesses(E0(0)),
                                                      thicknesses(E0(1)),
                                                      thicknesses(E1(0)),
                                                      thicknesses(E1(1)));


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
                           d_hat);

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
                   });
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
