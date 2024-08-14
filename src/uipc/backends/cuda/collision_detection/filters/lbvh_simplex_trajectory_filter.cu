#include <collision_detection/filters/lbvh_simplex_trajectory_filter.h>
#include <muda/cub/device/device_select.h>
#include <muda/ext/eigen/log_proxy.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/geo/distance/distance_type.h>
#include <muda/ext/geo/distance.h>

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

    auto alpha = info.alpha();
    auto d_hat = info.d_hat();
    auto Ps    = info.positions();
    auto dxs   = info.displacements();
    auto Vs    = info.surf_vertices();
    auto Es    = info.surf_edges();
    auto Fs    = info.surf_triangles();

    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());

    if(alpha == 0.0f)
    {
        // build AABBs for points
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(Vs.size(),
                   [Vs    = Vs.viewer().name("V"),
                    Ps    = Ps.viewer().name("Ps"),
                    aabbs = point_aabbs.viewer().name("aabbs"),
                    d_hat = d_hat] __device__(int i) mutable
                   {
                       auto        vI  = Vs(i);
                       const auto& pos = Ps(vI);

                       AABB aabb;
                       aabb.extend(pos);

                       aabb.min().array() -= d_hat;
                       aabb.max().array() += d_hat;
                       aabbs(i) = aabb;
                   });

        // build AABBs for edges
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(Es.size(),
                   [Es    = Es.viewer().name("E"),
                    Ps    = Ps.viewer().name("Ps"),
                    aabbs = edge_aabbs.viewer().name("aabbs"),
                    d_hat = d_hat] __device__(int i) mutable
                   {
                       const auto& eI   = Es(i);
                       const auto& pos0 = Ps(eI[0]);
                       const auto& pos1 = Ps(eI[1]);

                       AABB aabb;
                       aabb.extend(pos0).extend(pos1);
                       aabb.min().array() -= d_hat;
                       aabb.max().array() += d_hat;
                       aabbs(i) = aabb;
                   });

        // build AABBs for triangles
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(Fs.size(),
                   [Fs    = Fs.viewer().name("F"),
                    Ps    = Ps.viewer().name("Ps"),
                    aabbs = triangle_aabbs.viewer().name("aabbs"),
                    d_hat = d_hat] __device__(int i) mutable
                   {
                       const auto& fI   = Fs(i);
                       const auto& pos0 = Ps(fI[0]);
                       const auto& pos1 = Ps(fI[1]);
                       const auto& pos2 = Ps(fI[2]);

                       AABB aabb;
                       aabb.extend(pos0).extend(pos1).extend(pos2);

                       aabb.min().array() -= d_hat;
                       aabb.max().array() += d_hat;
                       aabbs(i) = aabb;
                   });

        // query PT
        lbvh_PT.build(triangle_aabbs);
        auto PT_pairs = lbvh_PT.query(
            point_aabbs,
            [Vs    = Vs.viewer().name("Vs"),
             Fs    = Fs.viewer().name("Fs"),
             Ps    = Ps.viewer().name("Ps"),
             d_hat = d_hat] __device__(IndexT i, IndexT j)
            {
                // discard if the point is on the triangle
                auto        V = Vs(i);
                const auto& F = Fs(j);

                if(F[0] == V || F[1] == V || F[2] == V)
                    return false;


                Vector3 VP  = Ps(V);
                Vector3 FP0 = Ps(F[0]);
                Vector3 FP1 = Ps(F[1]);
                Vector3 FP2 = Ps(F[2]);
                Float   D;
                muda::distance::point_triangle_distance_unclassified(VP, FP0, FP1, FP2, D);

                if(D >= d_hat * d_hat)
                    return false;

                //cout << "PT: " << V << " " << F.transpose().eval()
                //     << " d: " << sqrt(D) << "d_hat: " << d_hat << "\n";

                return true;
            });

        // query EE
        lbvh_EE.build(edge_aabbs);
        auto EE_pairs = lbvh_EE.detect(
            [Es    = Es.viewer().name("Es"),
             Ps    = Ps.viewer().name("Ps"),
             d_hat = d_hat] __device__(IndexT i, IndexT j)
            {
                // discard if the edges shared same vertex
                const auto& E0 = Es(i);
                const auto& E1 = Es(j);

                if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                    return false;

                Vector3 EP0 = Ps(E0[0]);
                Vector3 EP1 = Ps(E0[1]);
                Vector3 EP2 = Ps(E1[0]);
                Vector3 EP3 = Ps(E1[1]);
                Float   D;
                muda::distance::edge_edge_distance_unclassified(EP0, EP1, EP2, EP3, D);

                if(D >= d_hat * d_hat)
                    return false;

                //cout << "EE: " << E0.transpose().eval() << " " << E1.transpose().eval()
                //     << " d: " << sqrt(D) << "d_hat: " << d_hat << "\n";

                return true;
            });

        candidate_PTs.resize(PT_pairs.size());
        candidate_EEs.resize(EE_pairs.size());

        // record the candidate pairs
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PT_pairs.size(),
                   [PT_pairs = PT_pairs.viewer().name("PT_pairs"),
                    candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                    Fs = Fs.viewer().name("Fs"),
                    Vs = Vs.viewer().name("Vs")] __device__(int i) mutable
                   {
                       auto&       PT   = candidate_PTs(i);
                       const auto& pair = PT_pairs(i);
                       PT[0]            = Vs(pair[0]);
                       PT.segment<3>(1) = Fs(pair[1]);
                   });

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(EE_pairs.size(),
                   [EE_pairs = EE_pairs.viewer().name("EE_pairs"),
                    candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                    Es = Es.viewer().name("Es")] __device__(int i) mutable
                   {
                       auto&       EE   = candidate_EEs(i);
                       const auto& pair = EE_pairs(i);
                       EE.segment<2>(0) = Es(pair[0]);
                       EE.segment<2>(2) = Es(pair[1]);
                   });
    }
    else  // alpha > 0
    {
        // build AABBs for points
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(Vs.size(),
                   [Vs    = Vs.viewer().name("V"),
                    dxs   = dxs.viewer().name("dx"),
                    Ps    = Ps.viewer().name("Ps"),
                    aabbs = point_aabbs.viewer().name("aabbs"),
                    alpha = alpha,
                    d_hat = d_hat] __device__(int i) mutable
                   {
                       auto        vI    = Vs(i);
                       const auto& pos   = Ps(vI);
                       Vector3     pos_t = pos + dxs(vI) * alpha;

                       AABB aabb;

                       aabb.extend(pos).extend(pos_t);

                       aabb.min().array() -= d_hat;
                       aabb.max().array() += d_hat;
                       aabbs(i) = aabb;
                   });

        // build AABBs for edges
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(Es.size(),
                   [Es    = Es.viewer().name("E"),
                    Ps    = Ps.viewer().name("Ps"),
                    aabbs = edge_aabbs.viewer().name("aabbs"),
                    dxs   = dxs.viewer().name("dx"),
                    alpha = alpha,
                    d_hat = d_hat] __device__(int i) mutable
                   {
                       auto        eI     = Es(i);
                       const auto& pos0   = Ps(eI[0]);
                       const auto& pos1   = Ps(eI[1]);
                       Vector3     pos0_t = pos0 + dxs(eI[0]) * alpha;
                       Vector3     pos1_t = pos1 + dxs(eI[1]) * alpha;

                       Vector3 max = pos0_t;
                       Vector3 min = pos0_t;

                       AABB aabb;

                       aabb.extend(pos0).extend(pos1).extend(pos0_t).extend(pos1_t);

                       aabb.min().array() -= d_hat;
                       aabb.max().array() += d_hat;
                       aabbs(i) = aabb;
                   });

        // build AABBs for triangles
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(Fs.size(),
                   [Fs    = Fs.viewer().name("F"),
                    Ps    = Ps.viewer().name("Ps"),
                    aabbs = triangle_aabbs.viewer().name("aabbs"),
                    dxs   = dxs.viewer().name("dx"),
                    alpha = alpha,
                    d_hat = d_hat] __device__(int i) mutable
                   {
                       auto        fI     = Fs(i);
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

                       aabb.min().array() -= d_hat;
                       aabb.max().array() += d_hat;
                       aabbs(i) = aabb;
                   });

        // query PT
        lbvh_PT.build(triangle_aabbs);
        auto PT_pairs =
            lbvh_PT.query(point_aabbs,
                          [Vs    = Vs.viewer().name("Vs"),
                           Fs    = Fs.viewer().name("Fs"),
                           Ps    = Ps.viewer().name("Ps"),
                           dxs   = dxs.viewer().name("dxs"),
                           d_hat = d_hat,
                           alpha = alpha] __device__(IndexT i, IndexT j)
                          {
                              // discard if the point is on the triangle
                              auto V = Vs(i);
                              auto F = Fs(j);

                              if(F[0] == V || F[1] == V || F[2] == V)
                                  return false;

                              Vector3 VP  = Ps(V);
                              Vector3 dVP = alpha * dxs(V);

                              Vector3 FP0 = Ps(F[0]);
                              Vector3 FP1 = Ps(F[1]);
                              Vector3 FP2 = Ps(F[2]);

                              Vector3 dFP0 = alpha * dxs(F[0]);
                              Vector3 dFP1 = alpha * dxs(F[1]);
                              Vector3 dFP2 = alpha * dxs(F[2]);

                              if(!muda::distance::point_triangle_ccd_broadphase(
                                     VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, d_hat))
                                  return false;


                              return true;
                          });

        // query EE
        lbvh_EE.build(edge_aabbs);
        auto EE_pairs = lbvh_EE.detect(
            [Es    = Es.viewer().name("Es"),
             Ps    = Ps.viewer().name("Ps"),
             dxs   = dxs.viewer().name("dxs"),
             d_hat = d_hat,
             alpha = alpha] __device__(IndexT i, IndexT j)
            {
                // discard if the edges shared same vertex
                auto E0 = Es(i);
                auto E1 = Es(j);

                if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                    return false;

                Vector3 EP0  = Ps(E0[0]);
                Vector3 EP1  = Ps(E0[1]);
                Vector3 dEP0 = alpha * dxs(E0[0]);
                Vector3 dEP1 = alpha * dxs(E0[1]);

                Vector3 EP2  = Ps(E1[0]);
                Vector3 EP3  = Ps(E1[1]);
                Vector3 dEP2 = alpha * dxs(E1[0]);
                Vector3 dEP3 = alpha * dxs(E1[1]);

                if(!muda::distance::edge_edge_ccd_broadphase(
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
                       d_hat))
                    return false;

                return true;
            });

        candidate_PTs.resize(PT_pairs.size());
        candidate_EEs.resize(EE_pairs.size());

        // record the candidate pairs
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(PT_pairs.size(),
                   [PT_pairs = PT_pairs.viewer().name("PT_pairs"),
                    candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                    Fs = Fs.viewer().name("Fs"),
                    Vs = Vs.viewer().name("Vs")] __device__(int i) mutable
                   {
                       auto& PT         = candidate_PTs(i);
                       auto  pair       = PT_pairs(i);
                       PT[0]            = Vs(pair[0]);
                       PT.segment<3>(1) = Fs(pair[1]);
                   });

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(EE_pairs.size(),
                   [EE_pairs = EE_pairs.viewer().name("EE_pairs"),
                    candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                    Es = Es.viewer().name("Es")] __device__(int i) mutable
                   {
                       auto& EE         = candidate_EEs(i);
                       auto  pair       = EE_pairs(i);
                       EE.segment<2>(0) = Es(pair[0]);
                       EE.segment<2>(2) = Es(pair[1]);
                   });
    }

    if constexpr(PrintDebugInfo)
    {
        std::vector<Vector4i> candidate_PTs_host;
        std::vector<Vector4i> candidate_EEs_host;

        candidate_PTs.copy_to(candidate_PTs_host);
        candidate_EEs.copy_to(candidate_EEs_host);

        // print the candidate pairs
        for(auto& PT : candidate_PTs_host)
        {
            std::cout << "PT: " << PT.transpose() << std::endl;
        }

        for(auto& EE : candidate_EEs_host)
        {
            std::cout << "EE: " << EE.transpose() << std::endl;
        }
    }
}

void LBVHSimplexTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    using namespace muda;


    if constexpr(PrintDebugInfo)
    {
        std::vector<Vector4i> candidate_PTs_host;
        std::vector<Vector4i> candidate_EEs_host;

        candidate_PTs.copy_to(candidate_PTs_host);
        candidate_EEs.copy_to(candidate_EEs_host);

        // print the candidate pairs
        std::cout << "candidate pairs:" << std::endl;
        for(auto& PT : candidate_PTs_host)
        {
            std::cout << "PT: " << PT.transpose() << std::endl;
        }

        for(auto& EE : candidate_EEs_host)
        {
            std::cout << "EE: " << EE.transpose() << std::endl;
        }
    }

    auto d_hat = info.d_hat();
    auto Ps    = info.positions();

    // we will classify the PT EE pairs into PTs, EEs, PEs, PPs

    // PT: point-triangle, only 1 possible PT per candidate PT
    temp_PTs.resize(candidate_PTs.size());
    PTs.resize(candidate_PTs.size());

    // EE: edge-edge, only 1 possible EE per candidate EE
    temp_EEs.resize(candidate_EEs.size());
    EEs.resize(candidate_EEs.size());


    // PE: point-edge
    // 3 possible PE per candidate PT
    SizeT PT_to_PE_max_count = candidate_PTs.size() * 3;
    // 4 possible PE per candidate EE
    SizeT EE_to_PE_max_count = candidate_EEs.size() * 4;

    SizeT max_PE_count = PT_to_PE_max_count + EE_to_PE_max_count;
    temp_PEs.resize(max_PE_count);
    PEs.resize(max_PE_count);

    // PP: point-point
    // 3 possible PP per candidate PT
    SizeT PT_to_PP_max_count = candidate_PTs.size() * 3;
    // 4 possible PP per candidate EE
    SizeT EE_to_PP_max_count = candidate_EEs.size() * 4;

    SizeT max_PP_count = PT_to_PP_max_count + EE_to_PP_max_count;
    temp_PPs.resize(max_PP_count);
    PPs.resize(max_PP_count);


    // always use the squared distance to avoid numerical issue
    Float D_hat = d_hat * d_hat;


    // 1) PT-> PT, 3PE, 3PP
    auto PE_offset = 0;
    auto PE_count  = PT_to_PE_max_count;
    auto PP_offset = 0;
    auto PP_count  = PT_to_PP_max_count;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_PTs.size(),
               [Ps            = Ps.viewer().name("Ps"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                PTs           = temp_PTs.viewer().name("PTs"),
                PEs   = temp_PEs.view(PE_offset, PE_count).viewer().name("PEs"),
                PPs   = temp_PPs.view(PP_offset, PP_count).viewer().name("PPs"),
                D_hat = D_hat] __device__(int i) mutable
               {
                   auto I3 = i * 3;

                   // Invalidate all the candidates
                   {
                       PTs(i).array() = -1;

                       PEs(I3 + 0).array() = -1;
                       PEs(I3 + 1).array() = -1;
                       PEs(I3 + 2).array() = -1;

                       PPs(I3 + 0).array() = -1;
                       PPs(I3 + 1).array() = -1;
                       PPs(I3 + 2).array() = -1;
                   }

                   Vector4i PT = candidate_PTs(i);
                   IndexT   P  = PT(0);
                   Vector3i T  = PT.tail<3>();

                   const auto& V  = Ps(PT(0));
                   const auto& F0 = Ps(PT(1));
                   const auto& F1 = Ps(PT(2));
                   const auto& F2 = Ps(PT(3));

                   auto dist_type =
                       muda::distance::point_triangle_distance_type(V, F0, F1, F2);

                   if(dist_type == distance::PointTriangleDistanceType::PT)
                   {
                       if constexpr(PrintDebugInfo)
                       {
                           cout << "PT->PT:" << PT.transpose().eval() << "\n";
                       }

                       Float D;
                       distance::point_triangle_distance(V, F0, F1, F2, D);

                       if(D < D_hat)
                           PTs(i) = PT;

                       return;
                   }

                   // if not, then it can be PT->PE or PT->PP

                   // 3 possible PE
                   const Vector3i PE[3] = {
                       {P, T(0), T(1)}, {P, T(1), T(2)}, {P, T(2), T(0)}};

                   for(int j = 0; j < 3; ++j)
                   {
                       auto& pe = PE[j];
                       auto  E0 = Ps(pe(0));
                       auto  E1 = Ps(pe(1));
                       auto  E2 = Ps(pe(2));

                       auto dist_type = distance::point_edge_distance_type(V, E0, E1);

                       if(dist_type == distance::PointEdgeDistanceType::PE)
                       {
                           Float D;
                           distance::point_edge_distance(V, E0, E1, D);

                           if(D < D_hat)
                           {
                               if constexpr(PrintDebugInfo)
                               {
                                   cout << "PT->PE:" << PT.transpose().eval()
                                        << "->" << P << "," << T(j) << "\n";
                               }

                               PEs(I3 + j) = pe;
                           }
                       }
                   }

                   // 3 possible PP
                   const Vector2i PP[3] = {{P, T(0)}, {P, T(1)}, {P, T(2)}};

                   for(int j = 0; j < 3; ++j)
                   {
                       auto& pp = PP[j];
                       auto  P0 = Ps(pp(0));
                       auto  P1 = Ps(pp(1));

                       Float D;
                       distance::point_point_distance(P0, P1, D);

                       if(D < D_hat)
                       {
                           PPs(I3 + j) = pp;
                       }
                   }
               });

    // 2) EE-> PT, 4PE, 4PP
    PE_offset += PE_count;
    PE_count = EE_to_PE_max_count;
    PP_offset += PP_count;
    PP_count = EE_to_PP_max_count;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_EEs.size(),
               [Ps            = Ps.viewer().name("Ps"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                EEs           = temp_EEs.viewer().name("EEs"),
                PEs   = temp_PEs.view(PE_offset, PE_count).viewer().name("PEs"),
                PPs   = temp_PPs.view(PP_offset, PP_count).viewer().name("PPs"),
                D_hat = D_hat] __device__(int i) mutable
               {
                   auto I4 = i * 4;

                   // Invalidate all the candidates
                   {
                       EEs(i).array() = -1;

                       PEs(I4 + 0).array() = -1;
                       PEs(I4 + 1).array() = -1;
                       PEs(I4 + 2).array() = -1;
                       PEs(I4 + 3).array() = -1;

                       PPs(I4 + 0).array() = -1;
                       PPs(I4 + 1).array() = -1;
                       PPs(I4 + 2).array() = -1;
                       PPs(I4 + 3).array() = -1;
                   }


                   Vector4i EE = candidate_EEs(i);

                   IndexT Ea0 = EE(0);
                   IndexT Ea1 = EE(1);
                   IndexT Eb0 = EE(2);
                   IndexT Eb1 = EE(3);

                   Vector3 E0 = Ps(Ea0);
                   Vector3 E1 = Ps(Ea1);
                   Vector3 E2 = Ps(Eb0);
                   Vector3 E3 = Ps(Eb1);

                   auto dist_type = distance::edge_edge_distance_type(E0, E1, E2, E3);

                   if(dist_type == distance::EdgeEdgeDistanceType::EE)
                   {
                       if constexpr(PrintDebugInfo)
                       {
                           cout << "EE->EE:" << EE.transpose().eval() << "\n";
                       }


                       Float D;
                       distance::edge_edge_distance(E0, E1, E2, E3, D);

                       if(D < D_hat)
                           EEs(i) = EE;

                       return;
                   }

                   // if not, then it can be EE->PE or EE->PP

                   // 4 possible PE
                   const Vector3i PE[4] = {{Ea0, Eb0, Eb1},
                                           {Ea1, Eb0, Eb1},
                                           {Eb0, Ea0, Ea1},
                                           {Eb1, Ea0, Ea1}};

                   for(int j = 0; j < 4; ++j)
                   {
                       auto& pe = PE[j];
                       auto  E0 = Ps(pe(0));
                       auto  E1 = Ps(pe(1));
                       auto  E2 = Ps(pe(2));

                       auto dist_type = distance::point_edge_distance_type(E0, E1, E2);

                       if(dist_type == distance::PointEdgeDistanceType::PE)
                       {
                           Float D;
                           distance::point_edge_distance(E0, E1, E2, D);

                           if(D < D_hat)
                           {
                               if constexpr(PrintDebugInfo)
                               {
                                   cout << "EE->PE:" << EE.transpose().eval()
                                        << "->" << pe.transpose().eval() << "\n";
                               }

                               PEs(I4 + j) = pe;
                           }
                       }
                   }

                   // 4 possible PP
                   const Vector2i PP[4] = {{Ea0, Eb0}, {Ea0, Eb1}, {Ea1, Eb0}, {Ea1, Eb1}};

                   for(int j = 0; j < 4; ++j)
                   {
                       auto& pp = PP[j];
                       auto  P0 = Ps(pp(0));
                       auto  P1 = Ps(pp(1));

                       Float D;
                       distance::point_point_distance(P0, P1, D);

                       if(D < D_hat)
                       {
                           if constexpr(PrintDebugInfo)
                           {
                               cout << "EE->PP:" << EE.transpose().eval()
                                    << "->" << pp.transpose().eval() << "\n";
                           }

                           PPs(I4 + j) = pp;
                       }
                   }
               });

    DeviceSelect().If(temp_PTs.data(),
                      PTs.data(),
                      selected.data(),
                      temp_PTs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector4i& pt)
                      { return pt(0) != -1; });

    int h_selected = selected;
    PTs.resize(h_selected);

    DeviceSelect().If(temp_EEs.data(),
                      EEs.data(),
                      selected.data(),
                      temp_EEs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector4i& ee)
                      { return ee(0) != -1; });

    h_selected = selected;
    EEs.resize(h_selected);

    DeviceSelect().If(temp_PEs.data(),
                      PEs.data(),
                      selected.data(),
                      temp_PEs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector3i& pe)
                      { return pe(0) != -1; });

    h_selected = selected;
    PEs.resize(h_selected);

    DeviceSelect().If(temp_PPs.data(),
                      PPs.data(),
                      selected.data(),
                      temp_PPs.size(),
                      [] CUB_RUNTIME_FUNCTION(const Vector2i& pp)
                      { return pp(0) != -1; });

    h_selected = selected;
    PPs.resize(h_selected);

    info.PTs(PTs);
    info.EEs(EEs);
    info.PEs(PEs);
    info.PPs(PPs);


    if constexpr(PrintDebugInfo)
    {
        std::vector<Vector4i> PTs_host;
        std::vector<Vector4i> EEs_host;
        std::vector<Vector3i> PEs_host;
        std::vector<Vector2i> PPs_host;

        PTs.copy_to(PTs_host);
        EEs.copy_to(EEs_host);
        PEs.copy_to(PEs_host);
        PPs.copy_to(PPs_host);

        std::cout << "classify result:" << std::endl;

        for(auto& PT : PTs_host)
        {
            std::cout << "PT: " << PT.transpose() << std::endl;
        }

        for(auto& EE : EEs_host)
        {
            std::cout << "EE: " << EE.transpose() << std::endl;
        }

        for(auto& PE : PEs_host)
        {
            std::cout << "PE: " << PE.transpose() << std::endl;
        }

        for(auto& PP : PPs_host)
        {
            std::cout << "PP: " << PP.transpose() << std::endl;
        }
    }
}

void LBVHSimplexTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace muda;

    tois.resize(candidate_PTs.size() + candidate_EEs.size());

    auto PT_tois = tois.view(0, candidate_PTs.size());
    auto EE_tois = tois.view(candidate_PTs.size(), candidate_EEs.size());


    // TODO: Codimension IPC need thickness property, later we will add it
    constexpr Float thickness = 0.0;
    // TODO: Now hard code the minimum separation coefficient
    // gap = eta * (dist2_cur - thickness * thickness) / (dist_cur + thickness);
    constexpr Float eta = 0.1;

    // TODO: Now hard code the maximum iteration
    constexpr SizeT max_iter = 1000;

    // large enough toi (>1)
    constexpr Float large_enough_toi = 1.1;

    // PT
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_PTs.size(),
               [PT_tois       = PT_tois.viewer().name("PT_tois"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                Ps            = info.positions().viewer().name("Ps"),
                dxs           = info.displacements().viewer().name("dxs"),
                alpha         = info.alpha(),
                d_hat         = info.d_hat(),
                eta,
                thickness,
                max_iter,
                large_enough_toi] __device__(int i) mutable
               {
                   auto& PT = candidate_PTs(i);
                   auto  V  = PT[0];
                   auto  F  = PT.segment<3>(1);

                   Vector3 VP  = Ps(V);
                   Vector3 dVP = alpha * dxs(V);

                   Vector3 FP0 = Ps(F[0]);
                   Vector3 FP1 = Ps(F[1]);
                   Vector3 FP2 = Ps(F[2]);

                   Vector3 dFP0 = alpha * dxs(F[0]);
                   Vector3 dFP1 = alpha * dxs(F[1]);
                   Vector3 dFP2 = alpha * dxs(F[2]);

                   Float toi = large_enough_toi;


                   bool faraway = !muda::distance::point_triangle_ccd_broadphase(
                       VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, d_hat);

                   if(faraway)
                   {
                       PT_tois(i) = toi;
                       return;
                   }

                   bool hit = muda::distance::point_triangle_ccd(
                       VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, eta, thickness, max_iter, toi);

                   if(!hit)
                       toi = large_enough_toi;

                   PT_tois(i) = toi;
               });

    // EE
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_EEs.size(),
               [EE_tois       = EE_tois.viewer().name("EE_tois"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                Ps            = info.positions().viewer().name("Ps"),
                dxs           = info.displacements().viewer().name("dxs"),
                alpha         = info.alpha(),
                d_hat         = info.d_hat(),
                eta,
                thickness,
                max_iter,
                large_enough_toi] __device__(int i) mutable
               {
                   auto& EE = candidate_EEs(i);
                   auto  E0 = EE.segment<2>(0);
                   auto  E1 = EE.segment<2>(2);

                   Vector3 EP0  = Ps(E0[0]);
                   Vector3 EP1  = Ps(E0[1]);
                   Vector3 dEP0 = alpha * dxs(E0[0]);
                   Vector3 dEP1 = alpha * dxs(E0[1]);

                   Vector3 EP2  = Ps(E1[0]);
                   Vector3 EP3  = Ps(E1[1]);
                   Vector3 dEP2 = alpha * dxs(E1[0]);
                   Vector3 dEP3 = alpha * dxs(E1[1]);

                   Float toi = large_enough_toi;

                   bool faraway = !muda::distance::edge_edge_ccd_broadphase(
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

                   bool hit = muda::distance::edge_edge_ccd(
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
