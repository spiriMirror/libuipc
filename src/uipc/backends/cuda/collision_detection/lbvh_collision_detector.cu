#include <collision_detection/lbvh_collision_detector.h>
#include <collision_detection/global_collision_detector.h>
#include <muda/ext/geo/distance.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<LBVHCollisionDetector>
{
  public:
    static U<LBVHCollisionDetector> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<LBVHCollisionDetector>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(LBVHCollisionDetector);

void LBVHCollisionDetector::do_build()
{
    m_impl.global_vertex_manager  = find<GlobalVertexManager>();
    m_impl.global_surface_manager = find<GlobalSurfaceManager>();
    m_impl.global_contact_manager = find<GlobalContactManager>();

    auto global_collision_detector = find<GlobalCollisionDetector>();

    // add the detector to the global collision detector
    global_collision_detector->add_detector(
        [this](GlobalCollisionDetector::DetectCandidateInfo& info)
        { m_impl.detect_candidates(info); });
}


void LBVHCollisionDetector::Impl::detect_candidates(GlobalCollisionDetector::DetectCandidateInfo& info)
{
    using namespace muda;
    auto alpha = info.alpha();
    auto d_hat = global_contact_manager->d_hat();
    auto Ps    = global_vertex_manager->positions();
    auto dxs   = global_vertex_manager->displacements();
    auto Vs    = global_surface_manager->surf_vertices();
    auto Es    = global_surface_manager->surf_edges();
    auto Fs    = global_surface_manager->surf_triangles();

    spdlog::info("d_hat:{}, alpha:{}", d_hat, alpha);

    point_aabbs.resize(Vs.size());
    triangle_aabbs.resize(Fs.size());
    edge_aabbs.resize(Es.size());

    // build AABBs for points
    ParallelFor()
        .kernel_name(__FUNCTION__ "-points")
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

                   Vector3 max = pos_t;
                   Vector3 min = pos_t;

                   max = max.cwiseMax(pos);
                   min = min.cwiseMin(pos);

                   AABB aabb{min, max};

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

                   max = max.cwiseMax(pos1_t);
                   min = min.cwiseMin(pos1_t);

                   max = max.cwiseMax(pos0);
                   min = min.cwiseMin(pos0);

                   max = max.cwiseMax(pos1);
                   min = min.cwiseMin(pos1);

                   AABB aabb{min, max};

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

                   Vector3 max = pos0_t;
                   Vector3 min = pos0_t;

                   max = max.cwiseMax(pos1_t);
                   min = min.cwiseMin(pos1_t);

                   max = max.cwiseMax(pos2_t);
                   min = min.cwiseMin(pos2_t);

                   max = max.cwiseMax(pos0);
                   min = min.cwiseMin(pos0);

                   max = max.cwiseMax(pos1);
                   min = min.cwiseMin(pos1);

                   max = max.cwiseMax(pos2);
                   min = min.cwiseMin(pos2);

                   AABB aabb{min, max};

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
         dxs   = dxs.viewer().name("dxs"),
         d_hat = d_hat,
         alpha = alpha] __device__(IndexT i, IndexT j)
        {
            // discard if the point is on the triangle
            auto V = Vs(i);
            auto F = Fs(j);

            if(F[0] == V || F[1] == V || F[2] == V)
            {
                return false;
            }

            if(alpha == 0.0)
            {
                Vector3 VP  = Ps(V);
                Vector3 FP0 = Ps(F[0]);
                Vector3 FP1 = Ps(F[1]);
                Vector3 FP2 = Ps(F[2]);
                Float   d;
                muda::distance::point_triangle_distance_unclassified(VP, FP0, FP1, FP2, d);
                if(d > d_hat * d_hat)
                    return false;
            }
            else
            {
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
            }

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
            {
                return false;
            }

            if(alpha == 0.0)
            {
                Vector3 EP0 = Ps(E0[0]);
                Vector3 EP1 = Ps(E0[1]);
                Vector3 EP2 = Ps(E1[0]);
                Vector3 EP3 = Ps(E1[1]);
                Float   d;
                muda::distance::edge_edge_distance_unclassified(EP0, EP1, EP2, EP3, d);

                if(d > d_hat * d_hat)
                    return false;
            }
            else
            {
                Vector3 EP0  = Ps(E0[0]);
                Vector3 EP1  = Ps(E0[1]);
                Vector3 dEP0 = alpha * dxs(E0[0]);
                Vector3 dEP1 = alpha * dxs(E0[1]);

                Vector3 EP2  = Ps(E1[0]);
                Vector3 EP3  = Ps(E1[1]);
                Vector3 dEP2 = alpha * dxs(E1[0]);
                Vector3 dEP3 = alpha * dxs(E1[1]);

                if(!muda::distance::edge_edge_ccd_broadphase(
                       EP0, EP1, dEP0, dEP1, EP2, EP3, dEP2, dEP3, d_hat))
                    return false;
            }

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

    info.candidate_PTs(candidate_PTs);
    info.candidate_EEs(candidate_EEs);

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
}  // namespace uipc::backend::cuda
