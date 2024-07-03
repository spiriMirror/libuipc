#include <collision_detection/lbvh_simplex_ccd_filter.h>
#include <collision_detection/global_dcd_filter.h>
#include <muda/ext/geo/distance.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/cub/device/device_reduce.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<LBVHSimplexCCDFilter>
{
  public:
    static U<LBVHSimplexCCDFilter> create(SimEngine& engine)
    {
        auto& info = engine.world().scene().info();

        return info["contact"]["enable"].get<bool>() ?
                   make_unique<LBVHSimplexCCDFilter>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(LBVHSimplexCCDFilter);

void LBVHSimplexCCDFilter::Impl::broadphase_ccd(SimplexCCDFilter::FilterInfo& info)
{
    using namespace muda;
    auto alpha = info.alpha();
    auto d_hat = info.d_hat();
    auto Ps    = info.positions();
    auto dxs   = info.displacements();
    auto Vs    = info.surf_vertices();
    auto Es    = info.surf_edges();
    auto Fs    = info.surf_triangles();

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

                   AABB aabb;

                   aabb.extend(pos).extend(pos_t);

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

                   AABB aabb;

                   aabb.extend(pos0).extend(pos1).extend(pos0_t).extend(pos1_t);

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
                   EP0, EP1, dEP0, dEP1, EP2, EP3, dEP2, dEP3, d_hat))
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
            std::cout << "CCD-PT: " << PT.transpose() << std::endl;
        }

        for(auto& EE : candidate_EEs_host)
        {
            std::cout << "CCD-EE: " << EE.transpose() << std::endl;
        }
    }
}

void LBVHSimplexCCDFilter::Impl::narrowphase_ccd(SimplexCCDFilter::FilterInfo& info)
{
    using namespace muda;

    tois.resize(candidate_PTs.size() + candidate_EEs.size());

    auto PT_tois = tois.view(0, candidate_PTs.size());
    auto EE_tois = tois.view(candidate_PTs.size(), candidate_EEs.size());


    constexpr Float eta = 0.1;
    // TODO: Codimension IPC need thickness property
    constexpr Float thickness = 0.0;
    // TODO: Now hard code the max iteration
    constexpr SizeT max_iter = 1000;

    // PT
    ParallelFor()
        .kernel_name(__FUNCTION__ "-PT")
        .apply(candidate_PTs.size(),
               [PT_tois       = PT_tois.viewer().name("PT_tois"),
                candidate_PTs = candidate_PTs.viewer().name("candidate_PTs"),
                Ps            = info.positions().viewer().name("Ps"),
                dxs           = info.displacements().viewer().name("dxs"),
                alpha         = info.alpha(),
                d_hat         = info.d_hat(),
                eta,
                thickness,
                max_iter] __device__(int i) mutable
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

                   Float toi = 1.1;  // large enough (>1)

                   bool hit = muda::distance::point_triangle_ccd(
                       VP, FP0, FP1, FP2, dVP, dFP0, dFP1, dFP2, eta, thickness, max_iter, toi);

                   if(!hit)
                       toi = 1.1;

                   PT_tois(i) = toi;
               });

    // EE
    ParallelFor()
        .kernel_name(__FUNCTION__ "-EE")
        .apply(candidate_EEs.size(),
               [EE_tois       = EE_tois.viewer().name("EE_tois"),
                candidate_EEs = candidate_EEs.viewer().name("candidate_EEs"),
                Ps            = info.positions().viewer().name("Ps"),
                dxs           = info.displacements().viewer().name("dxs"),
                alpha         = info.alpha(),
                d_hat         = info.d_hat(),
                eta,
                thickness,
                max_iter] __device__(int i) mutable
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

                   Float toi = 1.1;  // large enough (>1)

                   bool hit = muda::distance::edge_edge_ccd(
                       EP0, EP1, dEP0, dEP1, EP2, EP3, dEP2, dEP3, eta, thickness, max_iter, toi);

                   if(!hit)
                       toi = 1.1;

                   EE_tois(i) = toi;
               });

    // get min toi
    DeviceReduce().Min(tois.data(), info.toi().data(), tois.size());

    Float toi = -1;

    info.toi().copy_to(&toi);

    spdlog::info("toi:{}", toi);
}

void LBVHSimplexCCDFilter::do_filter_toi(SimplexCCDFilter::FilterInfo& info)
{
    m_impl.broadphase_ccd(info);
    m_impl.narrowphase_ccd(info);
}
}  // namespace uipc::backend::cuda
