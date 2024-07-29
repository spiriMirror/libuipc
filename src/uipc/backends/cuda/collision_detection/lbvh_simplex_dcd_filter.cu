#include <collision_detection/lbvh_simplex_dcd_filter.h>
#include <muda/cub/device/device_select.h>
#include <muda/ext/eigen/log_proxy.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/geo/distance/distance_type.h>
#include <muda/ext/geo/distance.h>

namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo = false;

REGISTER_SIM_SYSTEM(LBVHSimplexDCDFilter);

void LBVHSimplexDCDFilter::do_detect(SimplexDCDFilter::FilterInfo& info)
{
    m_impl.detect(info);
    m_impl.classify(info);
}

void LBVHSimplexDCDFilter::Impl::detect(SimplexDCDFilter::FilterInfo& info)
{
    using namespace muda;

    auto d_hat = info.d_hat();
    auto Ps    = info.positions();
    auto Vs    = info.surf_vertices();
    auto Es    = info.surf_edges();
    auto Fs    = info.surf_triangles();

    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());


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
               [PT_pairs      = PT_pairs.viewer().name("PT_pairs"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                Fs            = Fs.viewer().name("Fs"),
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
               [EE_pairs      = EE_pairs.viewer().name("EE_pairs"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                Es = Es.viewer().name("Es")] __device__(int i) mutable
               {
                   auto&       EE   = candidate_EEs(i);
                   const auto& pair = EE_pairs(i);
                   EE.segment<2>(0) = Es(pair[0]);
                   EE.segment<2>(2) = Es(pair[1]);
               });

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

void LBVHSimplexDCDFilter::Impl::classify(SimplexDCDFilter::FilterInfo& info)
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
    temp_PTs.fill(Vector4i::Ones() * -1);  // fill -1 to indicate invalid PT
    PTs.resize(candidate_PTs.size());

    // EE: edge-edge, only 1 possible EE per candidate EE
    temp_EEs.resize(candidate_EEs.size());
    temp_EEs.fill(Vector4i::Ones() * -1);  // fill -1 to indicate invalid EE
    EEs.resize(candidate_EEs.size());


    // PE: point-edge
    // 3 possible PE per candidate PT
    SizeT PT_to_PE_max_count = candidate_PTs.size() * 3;
    // 4 possible PE per candidate EE
    SizeT EE_to_PE_max_count = candidate_EEs.size() * 4;

    SizeT max_PE_count = PT_to_PE_max_count + EE_to_PE_max_count;
    temp_PEs.resize(max_PE_count);
    temp_PEs.fill(Vector3i::Ones() * -1);  // fill -1 to indicate invalid PE
    PEs.resize(max_PE_count);

    // PP: point-point
    // 3 possible PP per candidate PT
    SizeT PT_to_PP_max_count = candidate_PTs.size() * 3;
    // 4 possible PP per candidate EE
    SizeT EE_to_PP_max_count = candidate_EEs.size() * 4;

    SizeT max_PP_count = PT_to_PP_max_count + EE_to_PP_max_count;
    temp_PPs.resize(max_PP_count);
    temp_PPs.fill(Vector2i::Ones() * -1);  // fill -1 to indicate invalid PP
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

                       if constexpr(muda::RUNTIME_CHECK_ON)
                       {
                           Float D;
                           distance::point_triangle_distance(V, F0, F1, F2, D);
                           MUDA_ASSERT(D < D_hat,
                                       "[%d,%d,%d,%d] D(%f) < D_hat=(%f)",
                                       P,
                                       T(0),
                                       T(1),
                                       T(2),
                                       D,
                                       D_hat);
                       }

                       PTs(i) = PT;

                       return;
                   }

                   // if not, then it can be PT->PE or PT->PP

                   // 3 possible PE
                   Vector3i PE[3] = {{P, T(0), T(1)}, {P, T(1), T(2)}, {P, T(2), T(0)}};

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

                               PEs(3 * i + j) = pe;
                           }
                       }
                   }

                   // 3 possible PP
                   Vector2i PP[3] = {{P, T(0)}, {P, T(1)}, {P, T(2)}};

                   for(int j = 0; j < 3; ++j)
                   {
                       auto& pp = PP[j];
                       auto  P0 = Ps(pp(0));
                       auto  P1 = Ps(pp(1));

                       Float D;
                       distance::point_point_distance(P0, P1, D);

                       if(D < D_hat)
                       {
                           PPs(3 * i + j) = pp;
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

                       if constexpr(muda::RUNTIME_CHECK_ON)
                       {
                           Float D;
                           distance::edge_edge_distance(E0, E1, E2, E3, D);
                           MUDA_ASSERT(
                               D < D_hat, "[%d,%d,%d,%d] D(%f) < D_hat=(%f)", Ea0, Ea1, Eb0, Eb1, D, D_hat);
                       }

                       EEs(i) = EE;

                       return;
                   }

                   // if not, then it can be EE->PE or EE->PP

                   // 4 possible PE
                   Vector3i PE[4] = {{Ea0, Eb0, Eb1},
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

                               PEs(4 * i + j) = pe;
                           }
                       }
                   }

                   // 4 possible PP
                   Vector2i PP[4] = {{Ea0, Eb0}, {Ea0, Eb1}, {Ea1, Eb0}, {Ea1, Eb1}};

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

                           PPs(4 * i + j) = pp;
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
}  // namespace uipc::backend::cuda
