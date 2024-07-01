#include <collision_detection/lbvh_simplex_dcd_filter.h>
#include <muda/cub/device/device_select.h>
#include <muda/ext/eigen/log_proxy.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/geo/distance/distance_type.h>
#include <muda/ext/geo/distance.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<LBVHSimplexDCDFilter>
{
  public:
    static U<LBVHSimplexDCDFilter> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<LBVHSimplexDCDFilter>(engine) :
                   nullptr;
    }
};

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
        .kernel_name(__FUNCTION__ "-points")
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
        .kernel_name(__FUNCTION__ "-edges")
        .apply(Es.size(),
               [Es    = Es.viewer().name("E"),
                Ps    = Ps.viewer().name("Ps"),
                aabbs = edge_aabbs.viewer().name("aabbs"),
                d_hat = d_hat] __device__(int i) mutable
               {
                   auto        eI   = Es(i);
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
        .kernel_name(__FUNCTION__ "-triangles")
        .apply(Fs.size(),
               [Fs    = Fs.viewer().name("F"),
                Ps    = Ps.viewer().name("Ps"),
                aabbs = triangle_aabbs.viewer().name("aabbs"),
                d_hat = d_hat] __device__(int i) mutable
               {
                   auto        fI   = Fs(i);
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
    auto PT_pairs = lbvh_PT.query(point_aabbs,
                                  [Vs    = Vs.viewer().name("Vs"),
                                   Fs    = Fs.viewer().name("Fs"),
                                   Ps    = Ps.viewer().name("Ps"),
                                   d_hat = d_hat] __device__(IndexT i, IndexT j)
                                  {
                                      // discard if the point is on the triangle
                                      auto V = Vs(i);
                                      auto F = Fs(j);

                                      if(F[0] == V || F[1] == V || F[2] == V)
                                          return false;


                                      Vector3 VP  = Ps(V);
                                      Vector3 FP0 = Ps(F[0]);
                                      Vector3 FP1 = Ps(F[1]);
                                      Vector3 FP2 = Ps(F[2]);
                                      Float   d;
                                      muda::distance::point_triangle_distance_unclassified(
                                          VP, FP0, FP1, FP2, d);

                                      if(d > d_hat * d_hat)
                                          return false;

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
            auto E0 = Es(i);
            auto E1 = Es(j);

            if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                return false;

            Vector3 EP0 = Ps(E0[0]);
            Vector3 EP1 = Ps(E0[1]);
            Vector3 EP2 = Ps(E1[0]);
            Vector3 EP3 = Ps(E1[1]);
            Float   d;
            muda::distance::edge_edge_distance_unclassified(EP0, EP1, EP2, EP3, d);

            if(d > d_hat * d_hat)
                return false;

            return true;
        });

    candidate_PTs.resize(PT_pairs.size());
    candidate_EEs.resize(EE_pairs.size());

    // record the candidate pairs
    ParallelFor()
        .kernel_name(__FUNCTION__ "-record PT pairs")
        .apply(PT_pairs.size(),
               [PT_pairs      = PT_pairs.viewer().name("PT_pairs"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                Fs            = Fs.viewer().name("Fs"),
                Vs = Vs.viewer().name("Vs")] __device__(int i) mutable
               {
                   auto& PT         = candidate_PTs(i);
                   auto  pair       = PT_pairs(i);
                   PT[0]            = Vs(pair[0]);
                   PT.segment<3>(1) = Fs(pair[1]);
               });

    ParallelFor()
        .kernel_name(__FUNCTION__ "-record EE pairs")
        .apply(EE_pairs.size(),
               [EE_pairs      = EE_pairs.viewer().name("EE_pairs"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                Es = Es.viewer().name("Es")] __device__(int i) mutable
               {
                   auto& EE         = candidate_EEs(i);
                   auto  pair       = EE_pairs(i);
                   EE.segment<2>(0) = Es(pair[0]);
                   EE.segment<2>(2) = Es(pair[1]);
               });

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
    auto d_hat = info.d_hat();
    auto Ps    = info.positions();

    temp_PTs.resize(candidate_PTs.size(), Vector4i::Ones() * -1);
    PTs.resize(candidate_PTs.size());
    temp_EEs.resize(candidate_EEs.size(), Vector4i::Ones() * -1);
    EEs.resize(candidate_EEs.size());

    // PE:
    SizeT max_PE_count = candidate_PTs.size() + candidate_EEs.size();
    temp_PEs.resize(max_PE_count, Vector3i::Ones() * -1);
    PEs.resize(max_PE_count);
    // PP:
    SizeT max_PP_count = candidate_PTs.size() + candidate_EEs.size();
    temp_PPs.resize(max_PP_count, Vector2i::Ones() * -1);
    PPs.resize(max_PP_count);  // reserve enough space for PTs


    auto PE_offset = 0;
    auto PE_count  = candidate_PTs.size();
    auto PP_offset = 0;
    auto PP_count  = candidate_PTs.size();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_PTs.size(),
               [Ps            = Ps.viewer().name("Ps"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                PTs           = temp_PTs.viewer().name("PTs"),
                PEs = temp_PEs.view(PE_offset, PE_count).viewer().name("PEs"),
                PPs = temp_PPs.view(PP_offset, PP_count).viewer().name("PPs")] __device__(int i) mutable
               {
                   Vector4i PT = candidate_PTs(i);
                   IndexT   P  = PT(0);
                   Vector3i T  = PT.tail<3>();


                   auto V  = Ps(PT(0));
                   auto F0 = Ps(PT(1));
                   auto F1 = Ps(PT(2));
                   auto F2 = Ps(PT(3));

                   auto dist_type =
                       muda::distance::point_triangle_distance_type(V, F0, F1, F2);

                   switch(dist_type)
                   {
                       case muda::distance::PointTriangleDistanceType::PP_PT0:
                           PPs(i) = Vector2i(P, T(0));
                           break;
                       case muda::distance::PointTriangleDistanceType::PP_PT1:
                           PPs(i) = Vector2i(P, T(1));
                           break;
                       case muda::distance::PointTriangleDistanceType::PP_PT2:
                           PPs(i) = Vector2i(P, T(2));
                           break;
                       case muda::distance::PointTriangleDistanceType::PE_PT0T1:
                           PEs(i) = Vector3i(P, T(0), T(1));
                           break;
                       case muda::distance::PointTriangleDistanceType::PE_PT1T2:
                           PEs(i) = Vector3i(P, T(1), T(2));
                           break;
                       case muda::distance::PointTriangleDistanceType::PE_PT2T0:
                           PEs(i) = Vector3i(P, T(2), T(0));
                           break;
                       case muda::distance::PointTriangleDistanceType::PT:
                           PTs(i) = PT;
                           break;
                       default:
                           break;
                   }
               });

    PE_offset = PE_count;
    PE_count  = candidate_EEs.size();
    PP_offset = PP_count;
    PP_count  = candidate_EEs.size();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(candidate_EEs.size(),
               [Ps            = Ps.viewer().name("Ps"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                EEs           = temp_EEs.viewer().name("EEs"),
                PEs = temp_PEs.view(PE_offset, PE_count).viewer().name("PEs"),
                PPs = temp_PPs.view(PP_offset, PP_count).viewer().name("PPs")] __device__(int i) mutable
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

                   switch(dist_type)
                   {
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea0Eb0:
                           PPs(i) = Vector2i(Ea0, Eb0);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea0Eb1:
                           PPs(i) = Vector2i(Ea0, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Ea0Eb0Eb1:
                           PEs(i) = Vector3i(Ea0, Eb0, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea1Eb0:
                           PPs(i) = Vector2i(Ea1, Eb0);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PP_Ea1Eb1:
                           PPs(i) = Vector2i(Ea1, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Ea1Eb0Eb1:
                           PEs(i) = Vector3i(Ea1, Eb0, Eb1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Eb0Ea0Ea1:
                           PEs(i) = Vector3i(Eb0, Ea0, Ea1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::PE_Eb1Ea0Ea1:
                           PEs(i) = Vector3i(Eb1, Ea0, Ea1);
                           break;
                       case muda::distance::EdgeEdgeDistanceType::EE:
                           EEs(i) = EE;
                           break;
                       default:
                           break;
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
}
}  // namespace uipc::backend::cuda
