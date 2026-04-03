#include <sanity_check/sanity_checker.h>
#include <contact_system/global_contact_manager.h>
#include <global_geometry/global_vertex_manager.h>
#include <global_geometry/global_simplicial_surface_manager.h>
#include <collision_detection/info_stackless_bvh.h>
#include <utils/simplex_contact_mask_utils.h>
#include <utils/primitive_d_hat.h>
#include <utils/codim_thickness.h>
#include <utils/distance/point_point.h>
#include <utils/distance/point_edge.h>
#include <utils/distance/point_triangle.h>
#include <utils/distance/edge_edge.h>

namespace uipc::backend::cuda
{
class SimplicialSurfaceDistanceCheck final : public SanityChecker
{
  public:
    using SanityChecker::SanityChecker;

    class Impl
    {
      public:
        SimSystemSlot<GlobalContactManager>           contact_manager;
        SimSystemSlot<GlobalVertexManager>             vertex_manager;
        SimSystemSlot<GlobalSimplicialSurfaceManager>  surface_manager;

        InfoStacklessBVH              point_bvh;
        InfoStacklessBVH              edge_bvh;
        InfoStacklessBVH              tri_bvh;
        InfoStacklessBVH::QueryBuffer pp_qbuffer;
        InfoStacklessBVH::QueryBuffer pe_qbuffer;
        InfoStacklessBVH::QueryBuffer pt_qbuffer;
        InfoStacklessBVH::QueryBuffer ee_qbuffer;

        muda::DeviceBuffer<AABB>   point_aabbs;
        muda::DeviceBuffer<IndexT> point_bids;
        muda::DeviceBuffer<IndexT> point_cids;

        muda::DeviceBuffer<AABB>   edge_aabbs;
        muda::DeviceBuffer<IndexT> edge_bids;
        muda::DeviceBuffer<IndexT> edge_cids;

        muda::DeviceBuffer<AABB>   tri_aabbs;
        muda::DeviceBuffer<IndexT> tri_bids;
        muda::DeviceBuffer<IndexT> tri_cids;

        muda::DeviceBuffer<IndexT> codim_vert_indices;
        muda::DeviceBuffer<AABB>   codim_point_aabbs;
        muda::DeviceBuffer<IndexT> codim_point_bids;
        muda::DeviceBuffer<IndexT> codim_point_cids;

        muda::DeviceVar<IndexT> violation_count;

        void init();
        void check(CheckInfo& info);

        void build_point_aabbs();
        void build_edge_aabbs();
        void build_tri_aabbs();
        void build_codim_point_aabbs();
    };

  protected:
    void do_build(BuildInfo& info) override
    {
        m_impl.contact_manager = require<GlobalContactManager>();
        m_impl.vertex_manager  = require<GlobalVertexManager>();
        m_impl.surface_manager = require<GlobalSimplicialSurfaceManager>();
    }

    void do_init(InitInfo& info) override { m_impl.init(); }
    void do_check(CheckInfo& info) override { m_impl.check(info); }

  private:
    Impl m_impl;
};

void SimplicialSurfaceDistanceCheck::Impl::init()
{
    auto dims = vertex_manager->dimensions();
    auto Vs   = surface_manager->surf_vertices();

    std::vector<IndexT> h_vs(Vs.size());
    Vs.copy_to(h_vs.data());

    std::vector<IndexT> h_dims(dims.size());
    dims.copy_to(h_dims.data());

    std::vector<IndexT> h_codim;
    for(SizeT i = 0; i < h_vs.size(); ++i)
    {
        if(h_dims[h_vs[i]] <= 1)
            h_codim.push_back(i);
    }
    codim_vert_indices.resize(h_codim.size());
    if(!h_codim.empty())
        codim_vert_indices.view().copy_from(h_codim.data());
}

void SimplicialSurfaceDistanceCheck::Impl::build_point_aabbs()
{
    auto Vs     = surface_manager->surf_vertices();
    auto Ps     = vertex_manager->positions();
    auto d_hats = vertex_manager->d_hats();
    auto thick  = vertex_manager->thicknesses();
    auto bids   = vertex_manager->body_ids();
    auto cids   = vertex_manager->contact_element_ids();

    point_aabbs.resize(Vs.size());
    point_bids.resize(Vs.size());
    point_cids.resize(Vs.size());

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Vs.size(),
               [Vs = Vs.viewer().name("Vs"),
                Ps = Ps.viewer().name("Ps"),
                d_hats = d_hats.viewer().name("d_hats"),
                thick  = thick.viewer().name("thick"),
                bids   = bids.viewer().name("bids"),
                cids   = cids.viewer().name("cids"),
                aabbs  = point_aabbs.viewer().name("aabbs"),
                obids  = point_bids.viewer().name("obids"),
                ocids  = point_cids.viewer().name("ocids")] __device__(int i) mutable
               {
                   auto vI = Vs(i);
                   float expand =
                       static_cast<float>(point_dcd_expansion(d_hats(vI)) + thick(vI));

                   AABB aabb;
                   aabb.extend(Ps(vI).cast<float>());
                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;

                   aabbs(i) = aabb;
                   obids(i) = bids(vI);
                   ocids(i) = cids(vI);
               });
}

void SimplicialSurfaceDistanceCheck::Impl::build_edge_aabbs()
{
    auto Es     = surface_manager->surf_edges();
    auto Ps     = vertex_manager->positions();
    auto d_hats = vertex_manager->d_hats();
    auto thick  = vertex_manager->thicknesses();
    auto bids   = vertex_manager->body_ids();
    auto cids   = vertex_manager->contact_element_ids();

    edge_aabbs.resize(Es.size());
    edge_bids.resize(Es.size());
    edge_cids.resize(Es.size());

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Es.size(),
               [Es = Es.viewer().name("Es"),
                Ps = Ps.viewer().name("Ps"),
                d_hats = d_hats.viewer().name("d_hats"),
                thick  = thick.viewer().name("thick"),
                bids   = bids.viewer().name("bids"),
                cids   = cids.viewer().name("cids"),
                aabbs  = edge_aabbs.viewer().name("aabbs"),
                obids  = edge_bids.viewer().name("obids"),
                ocids  = edge_cids.viewer().name("ocids")] __device__(int i) mutable
               {
                   auto E = Es(i);
                   float expand = static_cast<float>(
                       edge_dcd_expansion(d_hats(E[0]), d_hats(E[1]))
                       + thick(E[0]) + thick(E[1]));

                   AABB aabb;
                   aabb.extend(Ps(E[0]).cast<float>());
                   aabb.extend(Ps(E[1]).cast<float>());
                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;

                   aabbs(i) = aabb;
                   obids(i) = bids(E[0]);
                   ocids(i) = cids(E[0]);
               });
}

void SimplicialSurfaceDistanceCheck::Impl::build_tri_aabbs()
{
    auto Fs     = surface_manager->surf_triangles();
    auto Ps     = vertex_manager->positions();
    auto d_hats = vertex_manager->d_hats();
    auto thick  = vertex_manager->thicknesses();
    auto bids   = vertex_manager->body_ids();
    auto cids   = vertex_manager->contact_element_ids();

    tri_aabbs.resize(Fs.size());
    tri_bids.resize(Fs.size());
    tri_cids.resize(Fs.size());

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Fs.size(),
               [Fs = Fs.viewer().name("Fs"),
                Ps = Ps.viewer().name("Ps"),
                d_hats = d_hats.viewer().name("d_hats"),
                thick  = thick.viewer().name("thick"),
                bids   = bids.viewer().name("bids"),
                cids   = cids.viewer().name("cids"),
                aabbs  = tri_aabbs.viewer().name("aabbs"),
                obids  = tri_bids.viewer().name("obids"),
                ocids  = tri_cids.viewer().name("ocids")] __device__(int i) mutable
               {
                   auto F = Fs(i);
                   float expand = static_cast<float>(
                       triangle_dcd_expansion(d_hats(F[0]), d_hats(F[1]), d_hats(F[2]))
                       + thick(F[0]) + thick(F[1]) + thick(F[2]));

                   AABB aabb;
                   aabb.extend(Ps(F[0]).cast<float>());
                   aabb.extend(Ps(F[1]).cast<float>());
                   aabb.extend(Ps(F[2]).cast<float>());
                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;

                   aabbs(i) = aabb;
                   obids(i) = bids(F[0]);
                   ocids(i) = cids(F[0]);
               });
}

void SimplicialSurfaceDistanceCheck::Impl::build_codim_point_aabbs()
{
    auto Vs     = surface_manager->surf_vertices();
    auto Ps     = vertex_manager->positions();
    auto d_hats = vertex_manager->d_hats();
    auto thick  = vertex_manager->thicknesses();
    auto bids   = vertex_manager->body_ids();
    auto cids   = vertex_manager->contact_element_ids();

    auto N = codim_vert_indices.size();
    codim_point_aabbs.resize(N);
    codim_point_bids.resize(N);
    codim_point_cids.resize(N);

    if(N == 0)
        return;

    using namespace muda;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(N,
               [Vs = Vs.viewer().name("Vs"),
                Ps = Ps.viewer().name("Ps"),
                d_hats = d_hats.viewer().name("d_hats"),
                thick  = thick.viewer().name("thick"),
                bids   = bids.viewer().name("bids"),
                cids   = cids.viewer().name("cids"),
                codim_idx = codim_vert_indices.viewer().name("codim_idx"),
                aabbs  = codim_point_aabbs.viewer().name("aabbs"),
                obids  = codim_point_bids.viewer().name("obids"),
                ocids  = codim_point_cids.viewer().name("ocids")] __device__(int i) mutable
               {
                   auto surf_idx = codim_idx(i);
                   auto vI       = Vs(surf_idx);
                   float expand  = static_cast<float>(
                       point_dcd_expansion(d_hats(vI)) + thick(vI));

                   AABB aabb;
                   aabb.extend(Ps(vI).cast<float>());
                   aabb.min().array() -= expand;
                   aabb.max().array() += expand;

                   aabbs(i) = aabb;
                   obids(i) = bids(vI);
                   ocids(i) = cids(vI);
               });
}

void SimplicialSurfaceDistanceCheck::Impl::check(CheckInfo& info)
{
    using namespace muda;

    auto Vs = surface_manager->surf_vertices();
    auto Es = surface_manager->surf_edges();
    auto Fs = surface_manager->surf_triangles();
    auto Ps = vertex_manager->positions();
    auto thick = vertex_manager->thicknesses();
    auto cmts  = contact_manager->contact_mask_tabular();

    if(Vs.size() == 0)
        return;

    violation_count = 0;

    build_point_aabbs();
    if(Es.size() > 0)
        build_edge_aabbs();
    if(Fs.size() > 0)
        build_tri_aabbs();
    build_codim_point_aabbs();

    // Phase 1: CodimP-AllP
    if(codim_vert_indices.size() > 0 && Vs.size() > 0)
    {
        point_bvh.build(point_aabbs.view(), point_bids.view(), point_cids.view());
        point_bvh.query(
            codim_point_aabbs.view(),
            codim_point_bids.view(),
            codim_point_cids.view(),
            cmts,
            [] __device__(InfoStacklessBVH::NodePredInfo) { return true; },
            [Vs    = Vs.viewer().name("Vs"),
             Ps    = Ps.cviewer().name("Ps"),
             thick = thick.cviewer().name("thick"),
             codim_idx = codim_vert_indices.viewer().name("codim_idx"),
             violation = violation_count.viewer().name("violation")] __device__(
                InfoStacklessBVH::LeafPredInfo leaf)
            {
                auto vi_codim = Vs(codim_idx(leaf.i));
                auto vi_all   = Vs(leaf.j);

                if(vi_codim == vi_all)
                    return false;
                if(leaf.bid_i == leaf.bid_j && leaf.bid_i != static_cast<IndexT>(-1))
                    return false;

                Float thickness  = thick(vi_codim) + thick(vi_all);
                Float thickness2 = thickness * thickness;
                Float D;
                distance::point_point_distance2(Ps(vi_codim), Ps(vi_all), D);

                if(D <= thickness2)
                    muda::atomic_add(violation.data(), IndexT(1));
                return false;
            },
            pp_qbuffer);
    }

    // Phase 2: CodimP-AllE
    if(codim_vert_indices.size() > 0 && Es.size() > 0)
    {
        edge_bvh.build(edge_aabbs.view(), edge_bids.view(), edge_cids.view());
        edge_bvh.query(
            codim_point_aabbs.view(),
            codim_point_bids.view(),
            codim_point_cids.view(),
            cmts,
            [] __device__(InfoStacklessBVH::NodePredInfo) { return true; },
            [Vs    = Vs.viewer().name("Vs"),
             Es    = Es.viewer().name("Es"),
             Ps    = Ps.cviewer().name("Ps"),
             thick = thick.cviewer().name("thick"),
             codim_idx = codim_vert_indices.viewer().name("codim_idx"),
             violation = violation_count.viewer().name("violation")] __device__(
                InfoStacklessBVH::LeafPredInfo leaf)
            {
                auto vi_p = Vs(codim_idx(leaf.i));
                auto E    = Es(leaf.j);

                if(vi_p == E[0] || vi_p == E[1])
                    return false;

                Float thickness  = thick(vi_p) + thick(E[0]);
                Float thickness2 = thickness * thickness;
                Float D;
                distance::point_edge_distance2(Ps(vi_p), Ps(E[0]), Ps(E[1]), D);

                if(D <= thickness2)
                    muda::atomic_add(violation.data(), IndexT(1));
                return false;
            },
            pe_qbuffer);
    }

    // Phase 3: AllP-AllT
    if(Vs.size() > 0 && Fs.size() > 0)
    {
        tri_bvh.build(tri_aabbs.view(), tri_bids.view(), tri_cids.view());
        tri_bvh.query(
            point_aabbs.view(),
            point_bids.view(),
            point_cids.view(),
            cmts,
            [] __device__(InfoStacklessBVH::NodePredInfo) { return true; },
            [Vs    = Vs.viewer().name("Vs"),
             Fs    = Fs.viewer().name("Fs"),
             Ps    = Ps.cviewer().name("Ps"),
             thick = thick.cviewer().name("thick"),
             violation = violation_count.viewer().name("violation")] __device__(
                InfoStacklessBVH::LeafPredInfo leaf)
            {
                auto vi_p = Vs(leaf.i);
                auto F    = Fs(leaf.j);

                if(vi_p == F[0] || vi_p == F[1] || vi_p == F[2])
                    return false;

                Float thickness  = thick(vi_p) + thick(F[0]);
                Float thickness2 = thickness * thickness;
                Float D;
                distance::point_triangle_distance2(
                    Ps(vi_p), Ps(F[0]), Ps(F[1]), Ps(F[2]), D);

                if(D <= thickness2)
                    muda::atomic_add(violation.data(), IndexT(1));
                return false;
            },
            pt_qbuffer);
    }

    // Phase 4: AllE-AllE
    if(Es.size() > 1)
    {
        if(codim_vert_indices.size() == 0)
            edge_bvh.build(edge_aabbs.view(), edge_bids.view(), edge_cids.view());

        edge_bvh.detect(
            cmts,
            [] __device__(InfoStacklessBVH::NodePredInfo) { return true; },
            [Es    = Es.viewer().name("Es"),
             Ps    = Ps.cviewer().name("Ps"),
             thick = thick.cviewer().name("thick"),
             violation = violation_count.viewer().name("violation")] __device__(
                InfoStacklessBVH::LeafPredInfo leaf)
            {
                auto E0 = Es(leaf.i);
                auto E1 = Es(leaf.j);

                if(E0[0] == E1[0] || E0[0] == E1[1]
                   || E0[1] == E1[0] || E0[1] == E1[1])
                    return false;

                Float thickness  = thick(E0[0]) + thick(E1[0]);
                Float thickness2 = thickness * thickness;
                Float D;
                distance::edge_edge_distance2(
                    Ps(E0[0]), Ps(E0[1]), Ps(E1[0]), Ps(E1[1]), D);

                if(D <= thickness2)
                    muda::atomic_add(violation.data(), IndexT(1));
                return false;
            },
            ee_qbuffer);
    }

    IndexT h_violation = violation_count;
    if(h_violation > 0)
    {
        logger::error("GPU SanityCheck: {} thickness violation(s) detected "
                      "(frame={}, newton={})",
                      h_violation,
                      info.frame(),
                      info.newton_iter());
    }
}

REGISTER_SIM_SYSTEM(SimplicialSurfaceDistanceCheck);
}  // namespace uipc::backend::cuda
