#include <active_set_system/global_active_set_manager.h>
#include <active_set_system/active_set_reporter.h>
#include <utils/codim_thickness.h>
#include <utils/primitive_d_hat.h>
#include <utils/distance/distance_flagged.h>
#include <sim_engine.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalActiveSetManager);

void GlobalActiveSetManager::do_build()
{
    const auto& config = world().scene().config();

    m_impl.global_trajectory_filter = require<GlobalTrajectoryFilter>();
    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = require<GlobalSimplicialSurfaceManager>();
}

void GlobalActiveSetManager::init()
{
    m_impl.simplex_trajectory_filter =
        m_impl.global_trajectory_filter->find<SimplexTrajectoryFilter>();
    m_impl.vertex_half_plane_trajectory_filter =
        m_impl.global_trajectory_filter->find<VertexHalfPlaneTrajectoryFilter>();

    m_impl.init(world());
}

void GlobalActiveSetManager::Impl::init(WorldVisitor& world)
{
    mu             = 0.0;
    mu_scale       = 0.1;
    decay_factor   = 0.5;
    toi_threshold  = 0.001;
    energy_enabled = true;
}

void GlobalActiveSetManager::Impl::filter_active()
{
    using namespace muda;
    auto filter = [&](DeviceBuffer<int>& cnt)
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(cnt.size(),
                   [cnt       = cnt.viewer().name("cnt"),
                    large_cnt = static_cast<int>(
                        floor(log(1e-20) / log(decay_factor)))] __device__(int i) mutable
                   {
                       if(cnt(i) >= 1)
                           cnt(i) = large_cnt;
                   });
    };

    filter(PT_cnt);
    filter(EE_cnt);
}

void GlobalActiveSetManager::Impl::update_active_set()
{
    using namespace muda;

    auto merge = [&](DeviceBuffer<Vector2i>&      idx,
                     DeviceBuffer<Float>&         lambda,
                     DeviceBuffer<int>&           cnt,
                     const CBufferView<Vector2i>& new_idx,
                     const CBufferView<Float>&    tois)
    {
        const auto N0 = idx.size();
        const auto N  = idx.size() + new_idx.size();

        loose_resize(ij_hash_input, N);
        loose_resize(ij_hash, N);
        loose_resize(sort_index_input, N);
        loose_resize(sort_index, N);
        loose_resize(offset, N);
        loose_resize(unique_flag, N);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [N0,
                    idx0      = idx.cviewer().name("idx0"),
                    idx1      = new_idx.cviewer().name("idx1"),
                    tois      = tois.cviewer().name("tois"),
                    cnt       = cnt.cviewer().name("cnt"),
                    ij_hash   = ij_hash_input.viewer().name("ij_hash"),
                    sort_idx  = sort_index_input.viewer().name("sort_idx"),
                    threshold = static_cast<int>(
                        floor(log(0.01) / log(decay_factor)))] __device__(int i) mutable
                   {
                       if(i < N0 && cnt(i) <= threshold)
                       {
                           ij_hash(i) = (static_cast<int64_t>(idx0(i)(0)) << 32)
                                        + static_cast<int64_t>(idx0(i)(1));
                       }
                       else if(i >= N0 && tois(i - N0) < 1 - 1e-6)
                       {
                           ij_hash(i) = (static_cast<int64_t>(idx1(i - N0)(0)) << 32)
                                        + static_cast<int64_t>(idx1(i - N0)(1));
                       }
                       else
                       {
                           ij_hash(i) = -1;
                       }
                       sort_idx(i) = i;
                   });

        DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                    ij_hash.data(),
                                    sort_index_input.data(),
                                    sort_index.data(),
                                    N);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [ij_hash = ij_hash.cviewer().name("ij_hash"),
                    flag    = unique_flag.viewer().name("flag"),
                    sort_idx = sort_index.viewer().name("sort_idx")] __device__(int i) mutable
                   {
                       if(i >= 1 && ij_hash(i) == ij_hash(i - 1) && ij_hash(i) >= 0)
                       {
                           flag(i) = 0;
                           if(sort_idx(i) < sort_idx(i - 1))
                               sort_idx(i - 1) = sort_idx(i);
                       }
                       else
                       {
                           flag(i) = ij_hash(i) >= 0 ? 1 : 0;
                       }
                   });

        DeviceScan().ExclusiveSum(unique_flag.data(), offset.data(), N);

        loose_resize(tmp_idx, N0);
        loose_resize(tmp_lambda, N0);
        loose_resize(tmp_cnt, N0);
        tmp_idx.view().copy_from(idx);
        tmp_lambda.view().copy_from(lambda);
        tmp_cnt.view().copy_from(cnt);

        loose_resize(idx, N);
        loose_resize(lambda, N);
        loose_resize(cnt, N);

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(N,
                   [N,
                    N0,
                    flag       = unique_flag.cviewer().name("flag"),
                    offset     = offset.cviewer().name("offset"),
                    sort_idx   = sort_index.cviewer().name("sort_idx"),
                    tmp_idx    = tmp_idx.cviewer().name("tmp_idx"),
                    tmp_lambda = tmp_lambda.cviewer().name("tmp_lambda"),
                    tmp_cnt    = tmp_cnt.cviewer().name("tmp_cnt"),
                    idx1       = new_idx.cviewer().name("new_idx"),
                    new_idx    = idx.viewer().name("idx"),
                    new_lambda = lambda.viewer().name("lambda"),
                    new_cnt    = cnt.viewer().name("cnt"),
                    total_count = total_count.viewer().name("total_count")] __device__(int i) mutable
                   {
                       if(flag(i))
                       {
                           auto idx = sort_idx(i);
                           auto j   = offset(i);
                           if(idx < N0)
                           {
                               new_idx(j)    = tmp_idx(idx);
                               new_lambda(j) = tmp_lambda(idx);
                               new_cnt(j)    = tmp_cnt(idx);
                           }
                           else
                           {
                               new_idx(j)    = idx1(idx - N0);
                               new_lambda(j) = 0.0;
                               new_cnt(j)    = 0;
                           }
                       }
                       if(i == N - 1)
                       {
                           total_count = flag(i) + offset(i);
                       }
                   });

        int N1 = total_count;
        idx.resize(N1);
        lambda.resize(N1);
        cnt.resize(N1);
    };

    auto old_PT_size = PT_idx.size(), old_EE_size = EE_idx.size();

    merge(PT_idx,
          PT_lambda,
          PT_cnt,
          simplex_trajectory_filter->candidate_PTs(),
          simplex_trajectory_filter->toi_PTs());

    merge(EE_idx,
          EE_lambda,
          EE_cnt,
          simplex_trajectory_filter->candidate_EEs(),
          simplex_trajectory_filter->toi_EEs());

    logger::info("Active set update: PT={}->{}, EE={}->{}",
                 old_PT_size,
                 PT_idx.size(),
                 old_EE_size,
                 EE_idx.size());
}

void GlobalActiveSetManager::Impl::linearize_constraints()
{
    using namespace muda;
    auto thicknesses = global_vertex_manager->thicknesses();
    auto d_hats      = global_vertex_manager->d_hats();
    auto x           = non_penetrate_positions.view();
    auto vs          = global_simplicial_surface_manager->surf_vertices();
    auto edges       = global_simplicial_surface_manager->surf_edges();
    auto tris        = global_simplicial_surface_manager->surf_triangles();

    loose_resize(PTs, PT_idx.size());
    loose_resize(PT_d0, PT_idx.size());
    loose_resize(PT_d_grad, PT_idx.size());

    loose_resize(EEs, EE_idx.size());
    loose_resize(EE_d0, EE_idx.size());
    loose_resize(EE_d_grad, EE_idx.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PT_idx.size(),
               [thicknesses = thicknesses.cviewer().name("thicknesses"),
                d_hats      = d_hats.cviewer().name("d_hats"),
                PT_idx      = PT_idx.cviewer().name("PT_idx"),
                PT_lambda   = PT_lambda.cviewer().name("PT_lambda"),
                vs          = vs.cviewer().name("vs"),
                tris        = tris.cviewer().name("tris"),
                x           = x.cviewer().name("x"),
                PTs         = PTs.viewer().name("PTs"),
                d0          = PT_d0.viewer().name("d0"),
                d_grad = PT_d_grad.viewer().name("d_grad")] __device__(int idx) mutable
               {
                   Vector3i tri = tris(PT_idx(idx)[1]);
                   Vector4i PT(vs(PT_idx(idx)[0]), tri[0], tri[1], tri[2]);

                   PTs(idx) = PT;

                   const auto& P  = x(PT(0));
                   const auto& T0 = x(PT(1));
                   const auto& T1 = x(PT(2));
                   const auto& T2 = x(PT(3));

                   Float thickness = PT_thickness(thicknesses(PT(0)),
                                                  thicknesses(PT(1)),
                                                  thicknesses(PT(2)),
                                                  thicknesses(PT(3)));

                   Float d_hat = PT_d_hat(
                       d_hats(PT(0)), d_hats(PT(1)), d_hats(PT(2)), d_hats(PT(3)));

                   Vector4i flag = distance::point_triangle_distance_flag(P, T0, T1, T2);

                   Float D;
                   distance::point_triangle_distance2(flag, P, T0, T1, T2, D);
                   Float d = sqrt(D);

                   Vector12 GradD;
                   distance::point_triangle_distance2_gradient(flag, P, T0, T1, T2, GradD);
                   GradD /= 2 * d;
                   d_grad(idx) = GradD;

                   d -= GradD.segment<3>(0).dot(P);
                   d -= GradD.segment<3>(3).dot(T0);
                   d -= GradD.segment<3>(6).dot(T1);
                   d -= GradD.segment<3>(9).dot(T2);

                   d0(idx) = d - thickness - d_hat;
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EE_idx.size(),
               [thicknesses = thicknesses.cviewer().name("thicknesses"),
                d_hats      = d_hats.cviewer().name("d_hats"),
                EE_idx      = EE_idx.cviewer().name("EE_idx"),
                EE_lambda   = EE_lambda.cviewer().name("EE_lambda"),
                edges       = edges.cviewer().name("edges"),
                x           = x.cviewer().name("pos"),
                EEs         = EEs.viewer().name("EEs"),
                d0          = EE_d0.viewer().name("d0"),
                d_grad = EE_d_grad.viewer().name("d_grad")] __device__(int idx) mutable
               {
                   const Vector2i& e0 = edges(EE_idx(idx)[0]);
                   const Vector2i& e1 = edges(EE_idx(idx)[1]);
                   Vector4i        EE(e0[0], e0[1], e1[0], e1[1]);

                   EEs(idx) = EE;

                   const auto& E0 = x(EE(0));
                   const auto& E1 = x(EE(1));
                   const auto& E2 = x(EE(2));
                   const auto& E3 = x(EE(3));

                   Float thickness = EE_thickness(thicknesses(EE(0)),
                                                  thicknesses(EE(1)),
                                                  thicknesses(EE(2)),
                                                  thicknesses(EE(3)));

                   Float d_hat = EE_d_hat(
                       d_hats(EE(0)), d_hats(EE(1)), d_hats(EE(2)), d_hats(EE(3)));

                   Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

                   Float D;
                   distance::edge_edge_distance2(flag, E0, E1, E2, E3, D);
                   Float d = sqrt(D);

                   Vector12 GradD;
                   distance::edge_edge_distance2_gradient(flag, E0, E1, E2, E3, GradD);
                   GradD /= 2 * d;
                   d_grad(idx) = GradD;

                   d -= GradD.segment<3>(0).dot(E0);
                   d -= GradD.segment<3>(3).dot(E1);
                   d -= GradD.segment<3>(6).dot(E2);
                   d -= GradD.segment<3>(9).dot(E3);

                   d0(idx) = d - thickness - d_hat;
               });
}

void GlobalActiveSetManager::Impl::update_slack()
{
    using namespace muda;
    auto x_hat = global_vertex_manager->positions();

    loose_resize(PT_slack, PTs.size());
    loose_resize(EE_slack, EEs.size());

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PTs.size(),
               [mu        = mu,
                PTs       = PTs.cviewer().name("PTs"),
                x_hat     = x_hat.cviewer().name("x_hat"),
                PT_d_grad = PT_d_grad.cviewer().name("PT_d_grad"),
                PT_lambda = PT_lambda.cviewer().name("PT_lambda"),
                d0        = PT_d0.viewer().name("d0"),
                slacks = PT_slack.viewer().name("slacks")] __device__(int idx) mutable
               {
                   const Vector4i& PT     = PTs(idx);
                   const Vector12& d_grad = PT_d_grad(idx);

                   Float& slack   = slacks(idx);
                   Float  d       = d0(idx);
                   Float  lambda  = PT_lambda(idx);
                   Float  d_shift = 0.0;

                   d_shift += d_grad.segment<3>(0).dot(x_hat(PT(0)));
                   d_shift += d_grad.segment<3>(3).dot(x_hat(PT(1)));
                   d_shift += d_grad.segment<3>(6).dot(x_hat(PT(2)));
                   d_shift += d_grad.segment<3>(9).dot(x_hat(PT(3)));

                   if(d + d_shift - lambda / mu > 0)
                       slack = d + d_shift - lambda / mu;
                   else
                       slack = 0;

                   d -= slack + lambda / mu;

                   d0(idx) = d;
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EEs.size(),
               [mu        = mu,
                EEs       = EEs.cviewer().name("EEs"),
                x_hat     = x_hat.cviewer().name("x_hat"),
                EE_d_grad = EE_d_grad.cviewer().name("EE_d_grad"),
                EE_lambda = EE_lambda.cviewer().name("EE_lambda"),
                d0        = EE_d0.viewer().name("d0"),
                slacks = EE_slack.viewer().name("slacks")] __device__(int idx) mutable
               {
                   const Vector4i& EE     = EEs(idx);
                   const Vector12& d_grad = EE_d_grad(idx);

                   Float& slack   = slacks(idx);
                   Float  d       = d0(idx);
                   Float  lambda  = EE_lambda(idx);
                   Float  d_shift = 0.0;

                   d_shift += d_grad.segment<3>(0).dot(x_hat(EE(0)));
                   d_shift += d_grad.segment<3>(3).dot(x_hat(EE(1)));
                   d_shift += d_grad.segment<3>(6).dot(x_hat(EE(2)));
                   d_shift += d_grad.segment<3>(9).dot(x_hat(EE(3)));

                   if(d + d_shift - lambda / mu > 0)
                       slack = d + d_shift - lambda / mu;
                   else
                       slack = 0;

                   d -= slack + lambda / mu;

                   d0(idx) = d;
               });
}

void GlobalActiveSetManager::Impl::update_lambda()
{
    using namespace muda;
    auto hat_x = global_vertex_manager->positions();

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(PTs.size(),
               [mu        = mu,
                PTs       = PTs.cviewer().name("PTs"),
                hat_x     = hat_x.cviewer().name("hat_x"),
                PT_d_grad = PT_d_grad.cviewer().name("PT_d_grad"),
                d0        = PT_d0.cviewer().name("d0"),
                slacks    = PT_slack.cviewer().name("slacks"),
                PT_lambda = PT_lambda.viewer().name("PT_lambda"),
                PT_cnt = PT_cnt.viewer().name("PT_cnt")] __device__(int idx) mutable
               {
                   const Vector4i& PT     = PTs(idx);
                   const Vector12& d_grad = PT_d_grad(idx);

                   Float&  lambda = PT_lambda(idx);
                   IndexT& cnt    = PT_cnt(idx);

                   Float d       = d0(idx);
                   Float d_shift = 0.0;
                   Float slack   = slacks(idx);

                   d_shift += d_grad.segment<3>(0).dot(hat_x(PT(0)));
                   d_shift += d_grad.segment<3>(3).dot(hat_x(PT(1)));
                   d_shift += d_grad.segment<3>(6).dot(hat_x(PT(2)));
                   d_shift += d_grad.segment<3>(9).dot(hat_x(PT(3)));

                   d += slack + lambda / mu;

                   if(d + d_shift - lambda / mu > 0)
                   {
                       lambda = 0;
                       cnt += 1;
                   }
                   else
                   {
                       lambda -= (d + d_shift) * mu;
                       cnt = 0;
                   }
               });

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(EEs.size(),
               [mu        = mu,
                EEs       = EEs.cviewer().name("EEs"),
                hat_x     = hat_x.cviewer().name("hat_x"),
                EE_d_grad = EE_d_grad.cviewer().name("EE_d_grad"),
                d0        = EE_d0.cviewer().name("d0"),
                slacks    = EE_slack.cviewer().name("slacks"),
                EE_lambda = EE_lambda.viewer().name("EE_lambda"),
                EE_cnt = EE_cnt.viewer().name("EE_cnt")] __device__(int idx) mutable
               {
                   const Vector4i& EE     = EEs(idx);
                   const Vector12& d_grad = EE_d_grad(idx);

                   Float&  lambda = EE_lambda(idx);
                   IndexT& cnt    = EE_cnt(idx);

                   Float d = d0(idx);

                   Float d_shift = 0.0;
                   Float slack   = slacks(idx);

                   d_shift += d_grad.segment<3>(0).dot(hat_x(EE(0)));
                   d_shift += d_grad.segment<3>(3).dot(hat_x(EE(1)));
                   d_shift += d_grad.segment<3>(6).dot(hat_x(EE(2)));
                   d_shift += d_grad.segment<3>(9).dot(hat_x(EE(3)));

                   d += slack + lambda / mu;

                   if(d + d_shift - lambda / mu > 0)
                   {
                       lambda = 0;
                       cnt += 1;
                   }
                   else
                   {
                       lambda -= (d + d_shift) * mu;
                       cnt = 0;
                   }
               });
}

void GlobalActiveSetManager::Impl::record_non_penetrate_positions()
{
    auto x_hat = global_vertex_manager->positions();
    if(non_penetrate_positions.size() != x_hat.size())
        non_penetrate_positions.resize(x_hat.size());
    muda::BufferLaunch().copy<Vector3>(non_penetrate_positions.view(), std::as_const(x_hat));
}

void GlobalActiveSetManager::Impl::recover_non_penetrate_positions()
{
    for(auto&& [i, R] : enumerate(active_set_reporters.view()))
    {
        IndexT offset = 0, count = 0;
        R->report_vertex_offset_count(offset, count);
        NonPenetratePositionInfo info(this, offset, count);
        R->recover_non_penetrate(info);
    }
}

void GlobalActiveSetManager::Impl::advance_non_penetrate_positions(Float alpha)
{
    auto x_hat = global_vertex_manager->positions();
    muda::ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(non_penetrate_positions.size(),
               [xs     = non_penetrate_positions.viewer().name("xs"),
                x_hats = x_hat.cviewer().name("x_hat"),
                alpha  = alpha] __device__(int i) mutable
               {
                   // interpolate toward x_hat
                   const Vector3& x_hat = x_hats(i);
                   Vector3&       x     = xs(i);

                   x = x + (x_hat - x) * alpha;
               });
}

muda::CBufferView<Vector4i> GlobalActiveSetManager::PTs() const
{
    return m_impl.PTs.view();
}

muda::CBufferView<Float> GlobalActiveSetManager::PT_d0() const
{
    return m_impl.PT_d0.view();
}

muda::CBufferView<Vector12> GlobalActiveSetManager::PT_d_grad() const
{
    return m_impl.PT_d_grad.view();
}

muda::CBufferView<Float> GlobalActiveSetManager::PT_lambda() const
{
    return m_impl.PT_lambda.view();
}

muda::CBufferView<int> GlobalActiveSetManager::PT_cnt() const
{
    return m_impl.PT_cnt.view();
}

muda::CBufferView<Vector4i> GlobalActiveSetManager::EEs() const
{
    return m_impl.EEs.view();
}

muda::CBufferView<Float> GlobalActiveSetManager::EE_d0() const
{
    return m_impl.EE_d0.view();
}

muda::CBufferView<Vector12> GlobalActiveSetManager::EE_d_grad() const
{
    return m_impl.EE_d_grad.view();
}

muda::CBufferView<Float> GlobalActiveSetManager::EE_lambda() const
{
    return m_impl.EE_lambda.view();
}

muda::CBufferView<int> GlobalActiveSetManager::EE_cnt() const
{
    return m_impl.EE_cnt.view();
}

muda::CBufferView<Vector3> GlobalActiveSetManager::non_penetrate_positions() const
{
    return m_impl.non_penetrate_positions.view();
}

Float GlobalActiveSetManager::mu() const
{
    return m_impl.mu;
}

Float GlobalActiveSetManager::mu_scale() const
{
    return m_impl.mu_scale;
}

Float GlobalActiveSetManager::decay_factor() const
{
    return m_impl.decay_factor;
}

Float GlobalActiveSetManager::toi_threshold() const
{
    return m_impl.toi_threshold;
}

GlobalActiveSetManager::NonPenetratePositionInfo::NonPenetratePositionInfo(Impl* impl,
                                                                           SizeT offset,
                                                                           SizeT count) noexcept
    : m_impl(impl)
    , m_offset(offset)
    , m_count(count)
{
}

muda::BufferView<Vector3> GlobalActiveSetManager::NonPenetratePositionInfo::non_penetrate_positions() const noexcept
{
    return m_impl->non_penetrate_positions.view(m_offset, m_count);
}

void GlobalActiveSetManager::filter_active()
{
    m_impl.filter_active();
}

void GlobalActiveSetManager::update_active_set()
{
    Timer timer{"Update Active Set"};
    m_impl.update_active_set();
}

void GlobalActiveSetManager::linearize_constraints()
{
    m_impl.linearize_constraints();
}

void GlobalActiveSetManager::update_slack()
{
    m_impl.update_slack();
}

void GlobalActiveSetManager::update_lambda()
{
    Timer timer{"Update Lambda"};
    m_impl.update_lambda();
}

void GlobalActiveSetManager::record_non_penetrate_positions()
{
    m_impl.record_non_penetrate_positions();
}

void GlobalActiveSetManager::recover_non_penetrate_positions()
{
    m_impl.recover_non_penetrate_positions();
}

void GlobalActiveSetManager::advance_non_penetrate_positions(Float alpha)
{
    m_impl.advance_non_penetrate_positions(alpha);
}

void GlobalActiveSetManager::enable()
{
    m_impl.energy_enabled = true;
}

void GlobalActiveSetManager::disable()
{
    m_impl.energy_enabled = false;
}

bool GlobalActiveSetManager::is_enabled() const
{
    return m_impl.energy_enabled;
}

void GlobalActiveSetManager::mu(Float mu)
{
    m_impl.mu = mu;
}

void GlobalActiveSetManager::add_reporter(ActiveSetReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.active_set_reporters.register_subsystem(*reporter);
}
}  // namespace uipc::backend::cuda