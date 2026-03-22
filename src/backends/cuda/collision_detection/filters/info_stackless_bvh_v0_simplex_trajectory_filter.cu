#include <collision_detection/filters/info_stackless_bvh_v0_simplex_trajectory_filter.h>
#include <muda/cub/device/device_select.h>
#include <muda/ext/eigen/log_proxy.h>
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

REGISTER_SIM_SYSTEM(InfoStacklessBVHV0SimplexTrajectoryFilter);

void InfoStacklessBVHV0SimplexTrajectoryFilter::do_build(BuildInfo&)
{
    auto& config = world().scene().config();
    auto  method = config.find<std::string>("collision_detection/method");
    if(method->view()[0] != "info_stackless_bvh_v0")
    {
        throw SimSystemException("Info stackless BVH V0 unused");
    }
}

void InfoStacklessBVHV0SimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info);
}

void InfoStacklessBVHV0SimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void InfoStacklessBVHV0SimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

muda::CBufferView<Vector2i>
InfoStacklessBVHV0SimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_impl.candidate_AllP_AllT_pairs.view();
}

muda::CBufferView<Vector2i>
InfoStacklessBVHV0SimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_impl.candidate_AllE_AllE_pairs.view();
}

muda::CBufferView<Float> InfoStacklessBVHV0SimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    return m_impl.tois.view(pp_size + pe_size, pt_size);
}

muda::CBufferView<Float> InfoStacklessBVHV0SimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_impl.candidate_AllE_AllE_pairs.size();
    return m_impl.tois.view(pp_size + pe_size + pt_size, ee_size);
}

void InfoStacklessBVHV0SimplexTrajectoryFilter::Impl::detect(DetectInfo& info)
{
    using namespace muda;

    auto alpha   = info.alpha();
    auto Ps      = info.positions();
    auto dxs     = info.displacements();
    auto codimVs = info.codim_vertices();
    auto Vs      = info.surf_vertices();
    auto Es      = info.surf_edges();
    auto Fs      = info.surf_triangles();
    auto v2bs    = info.v2b();
    auto body_self_collisions = info.body_self_collision();
    auto contact_element_ids  = info.contact_element_ids();
    auto cmts = info.contact_mask_tabular();

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

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(codimVs.size(),
                   [codimVs = codimVs.viewer().name("codimVs"),
                    Ps      = Ps.viewer().name("Ps"),
                    dxs     = dxs.viewer().name("dxs"),
                    aabbs   = codim_point_aabbs.viewer().name("aabbs"),
                    v2bs    = v2bs.viewer().name("v2bs"),
                    contact_ids = contact_element_ids.viewer().name("contact_ids"),
                    bids = codim_point_bids.viewer().name("bids"),
                    cids = codim_point_cids.viewer().name("cids"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    alpha  = alpha] __device__(int i) mutable
                   {
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
                       bids(i) = v2bs(vI);
                       cids(i) = contact_ids(vI);
                   });
    }

    // build AABBs for surf vertices (including codim vertices)
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Vs.size(),
               [Vs          = Vs.viewer().name("V"),
                dxs         = dxs.viewer().name("dx"),
                Ps          = Ps.viewer().name("Ps"),
                aabbs       = point_aabbs.viewer().name("aabbs"),
                v2bs        = v2bs.viewer().name("v2bs"),
                contact_ids = contact_element_ids.viewer().name("contact_ids"),
                bids        = point_bids.viewer().name("bids"),
                cids        = point_cids.viewer().name("cids"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                d_hats      = info.d_hats().viewer().name("d_hats"),
                alpha       = alpha] __device__(int i) mutable
               {
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
               });

    // build AABBs for edges
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Es.size(),
               [Es          = Es.viewer().name("E"),
                Ps          = Ps.viewer().name("Ps"),
                aabbs       = edge_aabbs.viewer().name("aabbs"),
                v2bs        = v2bs.viewer().name("v2bs"),
                contact_ids = contact_element_ids.viewer().name("contact_ids"),
                bids        = edge_bids.viewer().name("bids"),
                cids        = edge_cids.viewer().name("cids"),
                dxs         = dxs.viewer().name("dx"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                d_hats      = info.d_hats().viewer().name("d_hats"),
                alpha       = alpha] __device__(int i) mutable
               {
                   auto eI = Es(i);

                   Float thickness =
                       edge_thickness(thicknesses(eI[0]), thicknesses(eI[1]));
                   Float d_hat_expansion =
                       edge_dcd_expansion(d_hats(eI[0]), d_hats(eI[1]));

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
               });

    // build AABBs for triangles
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(Fs.size(),
               [Fs          = Fs.viewer().name("F"),
                Ps          = Ps.viewer().name("Ps"),
                aabbs       = triangle_aabbs.viewer().name("aabbs"),
                v2bs        = v2bs.viewer().name("v2bs"),
                contact_ids = contact_element_ids.viewer().name("contact_ids"),
                bids        = triangle_bids.viewer().name("bids"),
                cids        = triangle_cids.viewer().name("cids"),
                dxs         = dxs.viewer().name("dx"),
                thicknesses = info.thicknesses().viewer().name("thicknesses"),
                d_hats      = info.d_hats().viewer().name("d_hats"),
                alpha       = alpha] __device__(int i) mutable
               {
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
                   bids(i)  = v2bs(fI[0]);
                   cids(i)  = contact_ids(fI[0]);
               });

    lbvh_E.build(edge_aabbs, edge_bids, edge_cids);
    lbvh_T.build(triangle_aabbs, triangle_bids, triangle_cids);

    if(codimVs.size() > 0)
    {
        // Use AllP to query CodimP
        {
            lbvh_CodimP.build(codim_point_aabbs, codim_point_bids, codim_point_cids);

            muda::KernelLabel label{__FUNCTION__, __FILE__, __LINE__};
            lbvh_CodimP.query(
                point_aabbs,
                point_bids,
                point_cids,
                cmts,
                [query_bids = point_bids.viewer().name("query_bids"),
                 query_cids = point_cids.viewer().name("query_cids"),
                 body_self_collision = body_self_collisions.viewer().name("body_self_collision"),
                 cmts = cmts.viewer().name("cmts")] __device__(InfoStacklessBVHV0::NodePredInfo info)
                {
                    constexpr IndexT invalid = static_cast<IndexT>(-1);
                    auto qbid = query_bids(info.query_id);
                    auto qcid = query_cids(info.query_id);
                    bool bid_cull = info.node_bid != invalid && qbid != invalid
                                    && qbid == info.node_bid
                                    && !body_self_collision(qbid);
                    bool cid_cull = info.node_cid != invalid && qcid != invalid
                                    && !cmts(qcid, info.node_cid);
                    return !(bid_cull || cid_cull);
                },
                [Vs      = Vs.viewer().name("Vs"),
                 codimVs = codimVs.viewer().name("codimVs"),

                 Ps          = Ps.viewer().name("Ps"),
                 dxs         = dxs.viewer().name("dxs"),
                 thicknesses = info.thicknesses().viewer().name("thicknesses"),
                 dimensions  = info.dimensions().viewer().name("dimensions"),
                 contact_element_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                 contact_mask_tabular = info.contact_mask_tabular().viewer().name("contact_mask_tabular"),
                 subscene_element_ids = info.subscene_element_ids().viewer().name("subscene_element_ids"),
                 subscene_mask_tabular = info.subscene_mask_tabular().viewer().name("subscene_mask_tabular"),
                 v2b = info.v2b().viewer().name("v2b"),
                 body_self_collision = info.body_self_collision().viewer().name("body_self_collision"),
                 d_hats = info.d_hats().viewer().name("d_hats"),
                 alpha  = alpha] __device__(IndexT i, IndexT j)
                {
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

                    auto body_i = v2b(V);
                    auto body_j = v2b(codimV);
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
                },
                candidate_AllP_CodimP_pairs);
        }

        // Use CodimP to query AllE
        {
            muda::KernelLabel label{__FUNCTION__, __FILE__, __LINE__};
            lbvh_E.query(
                codim_point_aabbs,
                codim_point_bids,
                codim_point_cids,
                cmts,
                [query_bids = codim_point_bids.viewer().name("query_bids"),
                 query_cids = codim_point_cids.viewer().name("query_cids"),
                 body_self_collision = body_self_collisions.viewer().name("body_self_collision"),
                 cmts = cmts.viewer().name("cmts")] __device__(InfoStacklessBVHV0::NodePredInfo info)
                {
                    constexpr IndexT invalid = static_cast<IndexT>(-1);
                    auto qbid = query_bids(info.query_id);
                    auto qcid = query_cids(info.query_id);
                    bool bid_cull = info.node_bid != invalid && qbid != invalid
                                    && qbid == info.node_bid
                                    && !body_self_collision(qbid);
                    bool cid_cull = info.node_cid != invalid && qcid != invalid
                                    && !cmts(qcid, info.node_cid);
                    return !(bid_cull || cid_cull);
                },
                [codimVs     = codimVs.viewer().name("Vs"),
                 Es          = Es.viewer().name("Es"),
                 Ps          = Ps.viewer().name("Ps"),
                 dxs         = dxs.viewer().name("dxs"),
                 thicknesses = info.thicknesses().viewer().name("thicknesses"),
                 contact_element_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
                 contact_mask_tabular = info.contact_mask_tabular().viewer().name("contact_mask_tabular"),
                 subscene_element_ids = info.subscene_element_ids().viewer().name("subscene_element_ids"),
                 subscene_mask_tabular = info.subscene_mask_tabular().viewer().name("subscene_mask_tabular"),
                 v2b = info.v2b().viewer().name("v2b"),
                 body_self_collision = info.body_self_collision().viewer().name("body_self_collision"),
                 d_hats = info.d_hats().viewer().name("d_hats"),
                 alpha  = alpha] __device__(IndexT i, IndexT j)
                {
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

                    auto body_i = v2b(codimV);
                    auto body_j = v2b(E[0]);
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
                },
                candidate_CodimP_AllE_pairs);
        }
    }

    // Use AllE to query AllE
    if(Es.size() > 0)
    {
        muda::KernelLabel label{__FUNCTION__, __FILE__, __LINE__};
        lbvh_E.detect(
            cmts,
            [query_bids = edge_bids.viewer().name("query_bids"),
             query_cids = edge_cids.viewer().name("query_cids"),
             body_self_collision = body_self_collisions.viewer().name("body_self_collision"),
             cmts = cmts.viewer().name("cmts")] __device__(InfoStacklessBVHV0::NodePredInfo info)
            {
                constexpr IndexT invalid = static_cast<IndexT>(-1);
                auto qbid = query_bids(info.query_id);
                auto qcid = query_cids(info.query_id);
                bool bid_cull = info.node_bid != invalid && qbid != invalid
                                && qbid == info.node_bid
                                && !body_self_collision(qbid);
                bool cid_cull = info.node_cid != invalid && qcid != invalid
                                && !cmts(qcid, info.node_cid);
                return !(bid_cull || cid_cull);
            },
            [Es          = Es.viewer().name("Es"),
             Ps          = Ps.viewer().name("Ps"),
             dxs         = dxs.viewer().name("dxs"),
             thicknesses = info.thicknesses().viewer().name("thicknesses"),
             contact_element_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
             contact_mask_tabular = info.contact_mask_tabular().viewer().name("contact_mask_tabular"),
             subscene_element_ids = info.subscene_element_ids().viewer().name("subscene_element_ids"),
             subscene_mask_tabular = info.subscene_mask_tabular().viewer().name("subscene_mask_tabular"),
             v2b = info.v2b().viewer().name("v2b"),
             body_self_collision = info.body_self_collision().viewer().name("body_self_collision"),
             d_hats = info.d_hats().viewer().name("d_hats"),
             alpha  = alpha] __device__(IndexT i, IndexT j)
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

                if(!allow_EE_contact(subscene_mask_tabular, scids))
                    return false;
                if(!allow_EE_contact(contact_mask_tabular, cids))
                    return false;

                if(E0[0] == E1[0] || E0[0] == E1[1] || E0[1] == E1[0] || E0[1] == E1[1])
                    return false;

                auto body_i = v2b(E0[0]);
                auto body_j = v2b(E1[0]);
                if(body_i == body_j && !body_self_collision(body_i))
                    return false;

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
            },
            candidate_AllE_AllE_pairs);
    }

    // Use AllP to query AllT
    if(Fs.size() > 0)
    {
        muda::KernelLabel label{__FUNCTION__, __FILE__, __LINE__};
        lbvh_T.query(
            point_aabbs,
            point_bids,
            point_cids,
            cmts,
            [query_bids = point_bids.viewer().name("query_bids"),
             query_cids = point_cids.viewer().name("query_cids"),
             body_self_collision = body_self_collisions.viewer().name("body_self_collision"),
             cmts = cmts.viewer().name("cmts")] __device__(InfoStacklessBVHV0::NodePredInfo info)
            {
                constexpr IndexT invalid = static_cast<IndexT>(-1);
                auto qbid = query_bids(info.query_id);
                auto qcid = query_cids(info.query_id);
                bool bid_cull = info.node_bid != invalid && qbid != invalid
                                && qbid == info.node_bid
                                && !body_self_collision(qbid);
                bool cid_cull = info.node_cid != invalid && qcid != invalid
                                && !cmts(qcid, info.node_cid);
                return !(bid_cull || cid_cull);
            },
            [Vs          = Vs.viewer().name("Vs"),
             Fs          = Fs.viewer().name("Fs"),
             Ps          = Ps.viewer().name("Ps"),
             dxs         = dxs.viewer().name("dxs"),
             thicknesses = info.thicknesses().viewer().name("thicknesses"),
             contact_element_ids = info.contact_element_ids().viewer().name("contact_element_ids"),
             contact_mask_tabular = info.contact_mask_tabular().viewer().name("contact_mask_tabular"),
             subscene_element_ids = info.subscene_element_ids().viewer().name("subscene_element_ids"),
             subscene_mask_tabular = info.subscene_mask_tabular().viewer().name("subscene_mask_tabular"),
             v2b = info.v2b().viewer().name("v2b"),
             body_self_collision = info.body_self_collision().viewer().name("body_self_collision"),
             d_hats = info.d_hats().viewer().name("d_hats"),
             alpha  = alpha] __device__(IndexT i, IndexT j)
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

                if(!allow_PT_contact(subscene_mask_tabular, scids))
                    return false;
                if(!allow_PT_contact(contact_mask_tabular, cids))
                    return false;

                if(F[0] == V || F[1] == V || F[2] == V)
                    return false;

                auto body_i = v2b(V);
                auto body_j = v2b(F[0]);
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
            },
            candidate_AllP_AllT_pairs);
    }
}

void InfoStacklessBVHV0SimplexTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    using namespace muda;

    auto positions = info.positions();

    SizeT N_PCoimP  = candidate_AllP_CodimP_pairs.size();
    SizeT N_CodimPE = candidate_CodimP_AllE_pairs.size();
    SizeT N_PTs     = candidate_AllP_AllT_pairs.size();
    SizeT N_EEs     = candidate_AllE_AllE_pairs.size();

    temp_PPs.resize(N_PCoimP + N_CodimPE + N_PTs + N_EEs);
    temp_PEs.resize(N_CodimPE + N_PTs + N_EEs);

    temp_PTs.resize(N_PTs);
    temp_EEs.resize(N_EEs);

    SizeT temp_PP_offset = 0;
    SizeT temp_PE_offset = 0;

    // AllP and CodimP
    if(N_PCoimP > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PCoimP);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(candidate_AllP_CodimP_pairs.size(),
                   [positions = positions.viewer().name("positions"),
                    PCodimP_pairs = candidate_AllP_CodimP_pairs.viewer().name("PP_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    codim_vertices = info.codim_vertices().viewer().name("codim_vertices"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    temp_PPs = PP_view.viewer().name("temp_PPs"),
                    d_hats = info.d_hats().viewer().name("d_hats")] __device__(int i) mutable
                   {
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
                               printf("[ISBVH][PP][low-dist] i=%d P=(%d,%d) D=%e range=(%e,%e) "
                                      "thickness=%e d_hat=%e\n",
                                      i, P0, P1, D, range.x(), range.y(), thickness, d_hat);
                           }
                       }

                       MUDA_ASSERT(D > range.x(),
                                   "Thickness Violated! D(%f) should be > D_range.x(%f), "
                                   "P=(%d,%d), thickness=%f, d_hat=%f",
                                   D, range.x(), P0, P1, thickness, d_hat);
                       if(!is_active_D(range, D))
                           return;

                       PP = {P0, P1};
                   });

        temp_PP_offset += N_PCoimP;
    }
    // CodimP and AllE
    if(N_CodimPE > 0)
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_CodimPE);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_CodimPE);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                candidate_CodimP_AllE_pairs.size(),
                [positions = positions.viewer().name("positions"),
                 CodimP_AllE_pairs = candidate_CodimP_AllE_pairs.viewer().name("PE_pairs"),
                 codim_veritces = info.codim_vertices().viewer().name("codim_vertices"),
                 surf_edges  = info.surf_edges().viewer().name("surf_edges"),
                 thicknesses = info.thicknesses().viewer().name("thicknesses"),
                 temp_PPs    = PP_view.viewer().name("temp_PPs"),
                 temp_PEs    = PE_view.viewer().name("temp_PEs"),
                 d_hats = info.d_hats().viewer().name("d_hats")] __device__(int i) mutable
                {
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
                            printf("[ISBVH][PE][low-dist] i=%d V-E=(%d,%d,%d) flag=(%d,%d,%d) "
                                   "D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                                   i, vIs(0), vIs(1), vIs(2), flag(0), flag(1), flag(2),
                                   D, range.x(), range.y(), thickness, d_hat);
                        }
                    }

                    MUDA_ASSERT(D > range.x(),
                                "Thickness Violated! D(%f) should be > D_range.x(%f), "
                                "V-E=(%d,%d,%d), flag=(%d,%d,%d), thickness=%f, d_hat=%f",
                                D, range.x(), vIs(0), vIs(1), vIs(2),
                                flag(0), flag(1), flag(2), thickness, d_hat);
                    if(!is_active_D(range, D))
                        return;

                    Vector3i offsets;
                    auto dim = distance::degenerate_point_edge(flag, offsets);

                    switch(dim)
                    {
                        case 2:
                        {
                            IndexT V0 = vIs(offsets(0));
                            IndexT V1 = vIs(offsets(1));
                            PP        = {V0, V1};
                        }
                        break;
                        case 3:
                        {
                            PE = vIs;
                        }
                        break;
                        default: {
                            MUDA_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
                        }
                        break;
                    }
                });

        temp_PP_offset += N_CodimPE;
        temp_PE_offset += N_CodimPE;
    }

    // AllP and AllT
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_PTs);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_PTs);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                candidate_AllP_AllT_pairs.size(),
                [positions = positions.viewer().name("Ps"),
                 PT_pairs = candidate_AllP_AllT_pairs.viewer().name("PT_pairs"),
                 surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                 surf_triangles = info.surf_triangles().viewer().name("surf_triangles"),
                 thicknesses = info.thicknesses().viewer().name("thicknesses"),
                 temp_PPs    = PP_view.viewer().name("temp_PPs"),
                 temp_PEs    = PE_view.viewer().name("temp_PEs"),
                 temp_PTs    = temp_PTs.viewer().name("temp_PTs"),
                 d_hats = info.d_hats().viewer().name("d_hats")] __device__(int i) mutable
                {
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
                            printf("[ISBVH][PT][low-dist] i=%d V-F=(%d,%d,%d,%d) "
                                   "flag=(%d,%d,%d,%d) D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                                   i, vIs(0), vIs(1), vIs(2), vIs(3),
                                   flag(0), flag(1), flag(2), flag(3),
                                   D, range.x(), range.y(), thickness, d_hat);
                        }
                    }

                    MUDA_ASSERT(
                        D > 0.0, "D=%f, V F = (%d,%d,%d,%d)", D, vIs(0), vIs(1), vIs(2), vIs(3));

                    MUDA_ASSERT(D > range.x(),
                                "Thickness Violated! D(%f) should be > D_range.x(%f), "
                                "V-F=(%d,%d,%d,%d), flag=(%d,%d,%d,%d), thickness=%f, d_hat=%f",
                                D, range.x(), vIs(0), vIs(1), vIs(2), vIs(3),
                                flag(0), flag(1), flag(2), flag(3), thickness, d_hat);
                    if(!is_active_D(range, D))
                        return;

                    Vector4i offsets;
                    auto dim = distance::degenerate_point_triangle(flag, offsets);

                    switch(dim)
                    {
                        case 2:
                        {
                            IndexT V0 = vIs(offsets(0));
                            IndexT V1 = vIs(offsets(1));
                            PP        = {V0, V1};
                        }
                        break;
                        case 3:
                        {
                            IndexT V0 = vIs(offsets(0));
                            IndexT V1 = vIs(offsets(1));
                            IndexT V2 = vIs(offsets(2));
                            PE        = {V0, V1, V2};
                        }
                        break;
                        case 4:
                        {
                            PT = vIs;
                        }
                        break;
                        default: {
                            MUDA_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
                        }
                        break;
                    }
                });

        temp_PP_offset += N_PTs;
        temp_PE_offset += N_PTs;
    }
    // AllE and AllE
    {
        auto PP_view = temp_PPs.view(temp_PP_offset, N_EEs);
        auto PE_view = temp_PEs.view(temp_PE_offset, N_EEs);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(
                candidate_AllE_AllE_pairs.size(),
                [positions = positions.viewer().name("Ps"),
                 rest_positions = info.rest_positions().viewer().name("rest_positions"),
                 EE_pairs = candidate_AllE_AllE_pairs.viewer().name("EE_pairs"),
                 surf_edges  = info.surf_edges().viewer().name("surf_edges"),
                 thicknesses = info.thicknesses().viewer().name("thicknesses"),
                 temp_PPs    = PP_view.viewer().name("temp_PPs"),
                 temp_PEs    = PE_view.viewer().name("temp_PEs"),
                 temp_EEs    = temp_EEs.viewer().name("temp_EEs"),
                 d_hats = info.d_hats().viewer().name("d_hats")] __device__(int i) mutable
                {
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
                            printf("[ISBVH][EE][low-dist] i=%d E-E=(%d,%d,%d,%d) "
                                   "flag=(%d,%d,%d,%d) D=%e range=(%e,%e) thickness=%e d_hat=%e\n",
                                   i, vIs(0), vIs(1), vIs(2), vIs(3),
                                   flag(0), flag(1), flag(2), flag(3),
                                   D, range.x(), range.y(), thickness, d_hat);
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
                        auto dim = distance::degenerate_edge_edge(flag, offsets);

                        switch(dim)
                        {
                            case 2:
                            {
                                IndexT V0 = vIs(offsets(0));
                                IndexT V1 = vIs(offsets(1));
                                PP        = {V0, V1};
                            }
                            break;
                            case 3:
                            {
                                IndexT V0 = vIs(offsets(0));
                                IndexT V1 = vIs(offsets(1));
                                IndexT V2 = vIs(offsets(2));
                                PE        = {V0, V1, V2};
                            }
                            break;
                            case 4:
                            {
                                EE = vIs;
                            }
                            break;
                            default: {
                                MUDA_ERROR_WITH_LOCATION("unexpected degenerate case dim=%d", dim);
                            }
                            break;
                        }
                    }
                });

        temp_PP_offset += N_EEs;
        temp_PE_offset += N_EEs;
    }

    UIPC_ASSERT(temp_PP_offset == temp_PPs.size(), "size mismatch");
    UIPC_ASSERT(temp_PE_offset == temp_PEs.size(), "size mismatch");

    {
        PPs.resize(temp_PPs.size());
        PEs.resize(temp_PEs.size());
        PTs.resize(temp_PTs.size());
        EEs.resize(temp_EEs.size());

        DeviceSelect().If(temp_PPs.data(),
                          PPs.data(),
                          selected_PP_count.data(),
                          temp_PPs.size(),
                          [] CUB_RUNTIME_FUNCTION(const Vector2i& PP)
                          { return PP(0) != -1; });

        DeviceSelect().If(temp_PEs.data(),
                          PEs.data(),
                          selected_PE_count.data(),
                          temp_PEs.size(),
                          [] CUB_RUNTIME_FUNCTION(const Vector3i& PE)
                          { return PE(0) != -1; });

        DeviceSelect().If(temp_PTs.data(),
                          PTs.data(),
                          selected_PT_count.data(),
                          temp_PTs.size(),
                          [] CUB_RUNTIME_FUNCTION(const Vector4i& PT)
                          { return PT(0) != -1; });

        DeviceSelect().If(temp_EEs.data(),
                          EEs.data(),
                          selected_EE_count.data(),
                          temp_EEs.size(),
                          [] CUB_RUNTIME_FUNCTION(const Vector4i& EE)
                          { return EE(0) != -1; });

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

void InfoStacklessBVHV0SimplexTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace muda;

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

    constexpr Float eta = 0.1;

    constexpr SizeT max_iter = 1000;

    constexpr Float large_enough_toi = 1.1;

    // AllP and CodimP
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(candidate_AllP_CodimP_pairs.size(),
                   [PP_tois = PP_tois.viewer().name("PP_tois"),
                    PCodimP_pairs = candidate_AllP_CodimP_pairs.viewer().name("PP_pairs"),
                    codim_vertices = info.codim_vertices().viewer().name("codim_vertices"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    positions = info.positions().viewer().name("Ps"),
                    dxs       = info.displacements().viewer().name("dxs"),
                    d_hats    = info.d_hats().viewer().name("d_hats"),
                    alpha     = info.alpha(),

                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
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
                   });
    }

    // CodimP and AllE
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(candidate_CodimP_AllE_pairs.size(),
                   [PE_tois = PE_tois.viewer().name("PE_tois"),
                    CodimP_AllE_pairs = candidate_CodimP_AllE_pairs.viewer().name("PE_pairs"),
                    codim_vertices = info.codim_vertices().viewer().name("codim_vertices"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    Ps         = info.positions().viewer().name("Ps"),
                    dxs        = info.displacements().viewer().name("dxs"),
                    d_hats     = info.d_hats().viewer().name("d_hats"),
                    alpha      = info.alpha(),
                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
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
                   });
    }

    // AllP and AllT
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(candidate_AllP_AllT_pairs.size(),
                   [PT_tois = PT_tois.viewer().name("PT_tois"),
                    PT_pairs = candidate_AllP_AllT_pairs.viewer().name("PT_pairs"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    surf_triangles = info.surf_triangles().viewer().name("surf_triangles"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    Ps     = info.positions().viewer().name("Ps"),
                    dxs    = info.displacements().viewer().name("dxs"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    alpha  = info.alpha(),
                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
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
                   });
    }

    // AllE and AllE
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(candidate_AllE_AllE_pairs.size(),
                   [EE_tois = EE_tois.viewer().name("EE_tois"),
                    EE_pairs = candidate_AllE_AllE_pairs.viewer().name("EE_pairs"),
                    surf_edges = info.surf_edges().viewer().name("surf_edges"),
                    thicknesses = info.thicknesses().viewer().name("thicknesses"),
                    Ps     = info.positions().viewer().name("Ps"),
                    dxs    = info.displacements().viewer().name("dxs"),
                    d_hats = info.d_hats().viewer().name("d_hats"),
                    alpha  = info.alpha(),
                    eta,
                    max_iter,
                    large_enough_toi] __device__(int i) mutable
                   {
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
                           EP0, EP1, EP2, EP3,
                           dEP0, dEP1, dEP2, dEP3,
                           d_hat + thickness);

                       if(faraway)
                       {
                           EE_tois(i) = toi;
                           return;
                       }

                       bool hit = distance::edge_edge_ccd(
                           EP0, EP1, EP2, EP3,
                           dEP0, dEP1, dEP2, dEP3,
                           eta, thickness, max_iter, toi);

                       if(!hit)
                           toi = large_enough_toi;

                       EE_tois(i) = toi;
                   });
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
