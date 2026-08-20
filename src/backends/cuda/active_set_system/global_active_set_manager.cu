#include <active_set_system/global_active_set_manager.h>
#include <active_set_system/al_stiffness_estimator.h>
#include <utils/distance/edge_edge_mollifier.h>
#include <active_set_system/active_set_reporter.h>
#include <utils/codim_thickness.h>
#include <utils/primitive_d_hat.h>
#include <utils/distance/distance_flagged.h>
#include <contact_system/contact_models/sym/vertex_half_plane_distance.inl>
#include <pipeline/al_ipc_pipeline_flag.h>
#include <uipc/common/log.h>
#include <implicit_geometry/half_plane_vertex_reporter.h>
#include <cuda_tool/cub.h>

namespace uipc::backend::cuda
{
namespace
{
    __global__ void filter_active_kernel(cuda_tool::BufferView<int> cnt,
                                         int                      large_cnt,
                                         int                      n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(cnt(i) >= 1)
            cnt(i) = large_cnt;
    }

    __global__ void update_active_set_k1_kernel(size_t                          N0,
                                                cuda_tool::CBufferView<Vector2i> idx0,
                                                cuda_tool::CBufferView<Vector2i> idx1,
                                                cuda_tool::CBufferView<Float> tois,
                                                cuda_tool::CBufferView<int>   cnt,
                                                cuda_tool::BufferView<int64_t> ij_hash,
                                                cuda_tool::BufferView<int> sort_idx,
                                                int                        threshold,
                                                int                        n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        if(i < N0 && abs(cnt(i)) <= threshold)
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
    }

    __global__ void update_active_set_k2_kernel(cuda_tool::CBufferView<int64_t> ij_hash,
                                                cuda_tool::BufferView<int> flag,
                                                cuda_tool::BufferView<int> sort_idx,
                                                int                        n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
    }

    __global__ void update_active_set_k3_kernel(size_t                         N,
                                                size_t                         N0,
                                                cuda_tool::CBufferView<int>    flag,
                                                cuda_tool::CBufferView<int>    offset,
                                                cuda_tool::CBufferView<int>    sort_idx,
                                                cuda_tool::CBufferView<Vector2i> tmp_idx,
                                                cuda_tool::CBufferView<Float> tmp_lambda,
                                                cuda_tool::CBufferView<int>   tmp_cnt,
                                                cuda_tool::CBufferView<Vector2i> idx1,
                                                cuda_tool::BufferView<Vector2i> new_idx,
                                                cuda_tool::BufferView<Float> new_lambda,
                                                cuda_tool::BufferView<int>   new_cnt,
                                                cuda_tool::Dense<int>        total_count,
                                                int                          n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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
    }

    __global__ void linearize_constraints_k1_kernel(
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        cuda_tool::CBufferView<Vector2i> PH_idx,
        cuda_tool::CBufferView<Vector3> x,
        cuda_tool::CBufferView<Vector3> plane_positions,
        cuda_tool::CBufferView<Vector3> plane_normals,
        cuda_tool::BufferView<int>      PHs,
        cuda_tool::BufferView<Float>    d0,
        cuda_tool::BufferView<Vector3>  d_grad,
        int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        int vI = PH_idx(idx)[0], hI = PH_idx(idx)[1];

        PHs(idx) = vI;

        const auto& P  = x(vI);
        const auto& hP = plane_positions(hI);
        const auto& hN = plane_normals(hI);

        Float thickness = thicknesses(vI);
        Float d_hat     = d_hats(vI);

        Float D;
        HalfPlaneD(D, P, hP, hN);
        D = sqrt(D);

        Vector3 GradD = hN;
        d_grad(idx)   = GradD;

        D -= GradD.dot(P);

        d0(idx) = D - thickness - d_hat;
    }

    __global__ void linearize_constraints_k2_kernel(
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        cuda_tool::CBufferView<Vector2i> PT_idx,
        cuda_tool::CBufferView<IndexT>  vs,
        cuda_tool::CBufferView<Vector3i> tris,
        cuda_tool::CBufferView<Vector3> x,
        cuda_tool::BufferView<Vector4i> PTs,
        cuda_tool::BufferView<Float>    d0,
        cuda_tool::BufferView<Vector12> d_grad,
        int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
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
        D = sqrt(D);

        Vector12 GradD;
        distance::point_triangle_distance2_gradient(flag, P, T0, T1, T2, GradD);
        GradD /= 2 * D;
        d_grad(idx) = GradD;

        D -= GradD.segment<3>(0).dot(P);
        D -= GradD.segment<3>(3).dot(T0);
        D -= GradD.segment<3>(6).dot(T1);
        D -= GradD.segment<3>(9).dot(T2);

        d0(idx) = D - thickness - d_hat;
    }

    __global__ void linearize_constraints_k3_kernel(
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<Float>   d_hats,
        cuda_tool::CBufferView<Vector2i> EE_idx,
        cuda_tool::CBufferView<Vector2i> edges,
        cuda_tool::CBufferView<Vector3> x,
        cuda_tool::BufferView<Vector4i> EEs,
        cuda_tool::BufferView<Float>    d0,
        cuda_tool::BufferView<Vector12> d_grad,
        cuda_tool::BufferView<Float>    lambda,
        cuda_tool::BufferView<int>      cnt,
        int                             large_cnt,
        int                             n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        Vector2i e0 = edges(EE_idx(idx)[0]), e1 = edges(EE_idx(idx)[1]);
        Vector4i EE(e0[0], e0[1], e1[0], e1[1]);

        EEs(idx) = EE;

        const auto& E0 = x(EE(0));
        const auto& E1 = x(EE(1));
        const auto& E2 = x(EE(2));
        const auto& E3 = x(EE(3));

        Float eps_x;
        distance::edge_edge_mollifier_threshold(E0, E1, E2, E3, 1e-6, eps_x);
        if(distance::need_mollify(E0, E1, E2, E3, eps_x))
        {
            cnt(idx)    = large_cnt;
            lambda(idx) = 0;
        }

        Float thickness = EE_thickness(thicknesses(EE(0)),
                                       thicknesses(EE(1)),
                                       thicknesses(EE(2)),
                                       thicknesses(EE(3)));

        Float d_hat = EE_d_hat(
            d_hats(EE(0)), d_hats(EE(1)), d_hats(EE(2)), d_hats(EE(3)));

        Vector4i flag = distance::edge_edge_distance_flag(E0, E1, E2, E3);

        Float D;
        distance::edge_edge_distance2(flag, E0, E1, E2, E3, D);
        D = sqrt(D);

        Vector12 GradD;
        distance::edge_edge_distance2_gradient(flag, E0, E1, E2, E3, GradD);
        GradD /= 2 * D;
        d_grad(idx) = GradD;

        D -= GradD.segment<3>(0).dot(E0);
        D -= GradD.segment<3>(3).dot(E1);
        D -= GradD.segment<3>(6).dot(E2);
        D -= GradD.segment<3>(9).dot(E3);

        d0(idx) = D - thickness - d_hat;
    }

    __global__ void update_slack_k1_kernel(cuda_tool::CBufferView<Float> mu_vertices,
                                           cuda_tool::CBufferView<int>   PHs,
                                           cuda_tool::CBufferView<Vector3> x_hat,
                                           cuda_tool::CBufferView<Vector3> PH_d_grad,
                                           cuda_tool::CBufferView<Float> PH_lambda,
                                           cuda_tool::BufferView<Float>  d0,
                                           cuda_tool::BufferView<Float>  slack,
                                           int                           n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto PH     = PHs(idx);
        auto mu     = mu_vertices(PH);
        auto d_grad = PH_d_grad(idx);
        auto d = d0(idx), lambda = PH_lambda(idx), d_shift = 0.0;
        d_shift += d_grad.dot(x_hat(PH));
        if(d + d_shift - lambda / mu > 0)
            slack(idx) = d + d_shift - lambda / mu;
        else
            slack(idx) = 0;
        d -= slack(idx) + lambda / mu;
        d0(idx) = d;
    }

    __global__ void update_slack_k2_kernel(cuda_tool::CBufferView<Float> mu_vertices,
                                           cuda_tool::CBufferView<Vector4i> PTs,
                                           cuda_tool::CBufferView<Vector3> x_hat,
                                           cuda_tool::CBufferView<Vector12> PT_d_grad,
                                           cuda_tool::CBufferView<Float> PT_lambda,
                                           cuda_tool::BufferView<Float>  d0,
                                           cuda_tool::BufferView<Float>  slack,
                                           int                           n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto PT = PTs(idx);
        auto mu = min(min(mu_vertices(PT(0)), mu_vertices(PT(1))),
                      min(mu_vertices(PT(2)), mu_vertices(PT(3))));
        auto d_grad = PT_d_grad(idx);
        auto d = d0(idx), lambda = PT_lambda(idx), d_shift = 0.0;
        d_shift += d_grad.segment<3>(0).dot(x_hat(PT(0)));
        d_shift += d_grad.segment<3>(3).dot(x_hat(PT(1)));
        d_shift += d_grad.segment<3>(6).dot(x_hat(PT(2)));
        d_shift += d_grad.segment<3>(9).dot(x_hat(PT(3)));
        if(d + d_shift - lambda / mu > 0)
            slack(idx) = d + d_shift - lambda / mu;
        else
            slack(idx) = 0;
        d -= slack(idx) + lambda / mu;
        d0(idx) = d;
    }

    __global__ void update_slack_k3_kernel(cuda_tool::CBufferView<Float> mu_vertices,
                                           cuda_tool::CBufferView<Vector4i> EEs,
                                           cuda_tool::CBufferView<Vector3> x_hat,
                                           cuda_tool::CBufferView<Vector12> EE_d_grad,
                                           cuda_tool::CBufferView<Float> EE_lambda,
                                           cuda_tool::BufferView<Float>  d0,
                                           cuda_tool::BufferView<Float>  slack,
                                           int                           n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto EE = EEs(idx);
        auto mu = min(min(mu_vertices(EE(0)), mu_vertices(EE(1))),
                      min(mu_vertices(EE(2)), mu_vertices(EE(3))));
        auto d_grad = EE_d_grad(idx);
        auto d = d0(idx), lambda = EE_lambda(idx), d_shift = 0.0;
        d_shift += d_grad.segment<3>(0).dot(x_hat(EE(0)));
        d_shift += d_grad.segment<3>(3).dot(x_hat(EE(1)));
        d_shift += d_grad.segment<3>(6).dot(x_hat(EE(2)));
        d_shift += d_grad.segment<3>(9).dot(x_hat(EE(3)));
        if(d + d_shift - lambda / mu > 0)
            slack(idx) = d + d_shift - lambda / mu;
        else
            slack(idx) = 0;
        d -= slack(idx) + lambda / mu;
        d0(idx) = d;
    }

    __global__ void update_lambda_k1_kernel(cuda_tool::CBufferView<Float> mu_vertices,
                                            cuda_tool::CBufferView<int>   PHs,
                                            cuda_tool::CBufferView<Vector3> x_hat,
                                            cuda_tool::CBufferView<Vector3> PH_d_grad,
                                            cuda_tool::CBufferView<Float> d0,
                                            cuda_tool::CBufferView<Float> slack,
                                            cuda_tool::BufferView<Float>  PH_lambda,
                                            cuda_tool::BufferView<int>    PH_cnt,
                                            int                           n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto vI     = PHs(idx);
        auto mu     = mu_vertices(vI);
        auto d_grad = PH_d_grad(idx);
        auto d = d0(idx), &lambda = PH_lambda(idx), d_shift = 0.0;
        auto& cnt = PH_cnt(idx);
        d_shift += d_grad.dot(x_hat(vI));
        d += slack(idx) + lambda / mu;
        if(d + d_shift - lambda / mu > 0)
        {
            lambda = 0;
            if(cnt >= 0)
                cnt++;
            else
                cnt--;
        }
        else
        {
            lambda -= (d + d_shift) * mu;
            if(cnt == 0 || cnt > 5)
                cnt = 0;
            else
                cnt = -1;
        }
    }

    __global__ void update_lambda_k2_kernel(cuda_tool::CBufferView<Float> mu_vertices,
                                            cuda_tool::CBufferView<Vector4i> PTs,
                                            cuda_tool::CBufferView<Vector3> x_hat,
                                            cuda_tool::CBufferView<Vector12> PT_d_grad,
                                            cuda_tool::CBufferView<Float> d0,
                                            cuda_tool::CBufferView<Float> slack,
                                            cuda_tool::BufferView<Float>  PT_lambda,
                                            cuda_tool::BufferView<int>    PT_cnt,
                                            int                           n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto  PT = PTs(idx);
        auto  mu = min(min(mu_vertices(PT(0)), mu_vertices(PT(1))),
                       min(mu_vertices(PT(2)), mu_vertices(PT(3))));
        auto  d_grad = PT_d_grad(idx);
        auto  d = d0(idx), &lambda = PT_lambda(idx), d_shift = 0.0;
        auto& cnt = PT_cnt(idx);
        d_shift += d_grad.segment<3>(0).dot(x_hat(PT(0)));
        d_shift += d_grad.segment<3>(3).dot(x_hat(PT(1)));
        d_shift += d_grad.segment<3>(6).dot(x_hat(PT(2)));
        d_shift += d_grad.segment<3>(9).dot(x_hat(PT(3)));
        d += slack(idx) + lambda / mu;
        if(d + d_shift - lambda / mu > 0)
        {
            lambda = 0;
            if(cnt >= 0)
                cnt++;
            else
                cnt--;
        }
        else
        {
            lambda -= (d + d_shift) * mu;
            if(cnt == 0 || cnt > 5)
                cnt = 0;
            else
                cnt = -1;
        }
    }

    __global__ void update_lambda_k3_kernel(cuda_tool::CBufferView<Float> mu_vertices,
                                            cuda_tool::CBufferView<Vector4i> EEs,
                                            cuda_tool::CBufferView<Vector3> x_hat,
                                            cuda_tool::CBufferView<Vector12> EE_d_grad,
                                            cuda_tool::CBufferView<Float> d0,
                                            cuda_tool::CBufferView<Float> slack,
                                            cuda_tool::BufferView<Float>  EE_lambda,
                                            cuda_tool::BufferView<int>    EE_cnt,
                                            int                           n)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= n)
            return;
        auto  EE = EEs(idx);
        auto  mu = min(min(mu_vertices(EE(0)), mu_vertices(EE(1))),
                       min(mu_vertices(EE(2)), mu_vertices(EE(3))));
        auto  d_grad = EE_d_grad(idx);
        auto  d = d0(idx), &lambda = EE_lambda(idx), d_shift = 0.0;
        auto& cnt = EE_cnt(idx);
        d_shift += d_grad.segment<3>(0).dot(x_hat(EE(0)));
        d_shift += d_grad.segment<3>(3).dot(x_hat(EE(1)));
        d_shift += d_grad.segment<3>(6).dot(x_hat(EE(2)));
        d_shift += d_grad.segment<3>(9).dot(x_hat(EE(3)));
        d += slack(idx) + lambda / mu;
        if(d + d_shift - lambda / mu > 0)
        {
            lambda = 0;
            if(cnt >= 0)
                cnt++;
            else
                cnt--;
        }
        else
        {
            lambda -= (d + d_shift) * mu;
            if(cnt == 0 || cnt > 5)
                cnt = 0;
            else
                cnt = -1;
        }
    }

    __global__ void advance_non_penetrate_positions_kernel(
        cuda_tool::BufferView<Vector3>  x,
        cuda_tool::CBufferView<Vector3> x_hat,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        x(i) = x(i) + (x_hat(i) - x(i)) * alpha;
    }
}  // namespace

REGISTER_SIM_SYSTEM(GlobalActiveSetManager);

void GlobalActiveSetManager::do_build()
{
    require<ALIPCPipelineFlag>();

    m_impl.global_trajectory_filter = require<GlobalTrajectoryFilter>();
    m_impl.global_vertex_manager    = require<GlobalVertexManager>();
    m_impl.global_simplicial_surface_manager = require<GlobalSimplicialSurfaceManager>();
    m_impl.half_plane = find<HalfPlane>();

    on_init_scene(
        [this]
        {
            m_impl.simplex_trajectory_filter =
                m_impl.global_trajectory_filter->find<SimplexTrajectoryFilter>();
            m_impl.vertex_half_plane_trajectory_filter =
                m_impl.global_trajectory_filter->find<VertexHalfPlaneTrajectoryFilter>();
        });
}

void GlobalActiveSetManager::Impl::init_mu()
{
    mu_vertices.resize(global_vertex_manager->positions().size());
    mu_vertices.view().fill(0.0);

    StiffnessEstimateInfo info{this};
    for(auto&& [i, R] : enumerate(stiffness_estimators.view()))
    {
        R->estimate_mu(info);
    }
}

void GlobalActiveSetManager::Impl::filter_active()
{
    using namespace cuda_tool;
    auto filter = [&](DeviceBuffer<int>& cnt)
    {
        int n = static_cast<int>(cnt.size());
        if(n > 0)
            filter_active_kernel<<<cuda_tool::best_grid_dim(n, filter_active_kernel),
                                   cuda_tool::best_block_dim(filter_active_kernel),
                                   0,
                                   nullptr>>>(cnt.view(), 1 << 30, n);
    };

    filter(PT_cnt);
    filter(EE_cnt);
}

void GlobalActiveSetManager::Impl::update_active_set()
{
    using namespace cuda_tool;

    auto merge = [&](DeviceBuffer<Vector2i>&      idx,
                     DeviceBuffer<Float>&         lambda,
                     DeviceBuffer<int>&           cnt,
                     const CBufferView<Vector2i>& new_idx,
                     const CBufferView<Float>&    tois)
    {
        const auto N0 = idx.size(), N = idx.size() + new_idx.size();
        loose_resize(ij_hash_input, N);
        loose_resize(ij_hash, N);
        loose_resize(sort_index_input, N);
        loose_resize(sort_index, N);
        loose_resize(offset, N);
        loose_resize(unique_flag, N);

        int n = static_cast<int>(N);
        if(n > 0)
            update_active_set_k1_kernel<<<
                cuda_tool::best_grid_dim(n, update_active_set_k1_kernel),
                cuda_tool::best_block_dim(update_active_set_k1_kernel),
                0,
                nullptr>>>(N0,
                           idx.cview(),
                           new_idx,
                           tois,
                           cnt.cview(),
                           ij_hash_input.view(),
                           sort_index_input.view(),
                           25,
                           n);

        DeviceRadixSort().SortPairs(ij_hash_input.data(),
                                    ij_hash.data(),
                                    sort_index_input.data(),
                                    sort_index.data(),
                                    N);

        if(n > 0)
            update_active_set_k2_kernel<<<
                cuda_tool::best_grid_dim(n, update_active_set_k2_kernel),
                cuda_tool::best_block_dim(update_active_set_k2_kernel),
                0,
                nullptr>>>(ij_hash.cview(), unique_flag.view(), sort_index.view(), n);

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

        total_count = 0;

        if(n > 0)
            update_active_set_k3_kernel<<<
                cuda_tool::best_grid_dim(n, update_active_set_k3_kernel),
                cuda_tool::best_block_dim(update_active_set_k3_kernel),
                0,
                nullptr>>>(N,
                           N0,
                           unique_flag.cview(),
                           offset.cview(),
                           sort_index.cview(),
                           tmp_idx.cview(),
                           tmp_lambda.cview(),
                           tmp_cnt.cview(),
                           new_idx,
                           idx.view(),
                           lambda.view(),
                           cnt.view(),
                           total_count.viewer(),
                           n);

        int N1 = total_count;
        idx.resize(N1);
        lambda.resize(N1);
        cnt.resize(N1);
    };

    auto old_PH_size = PH_idx.size(), old_PT_size = PT_idx.size(),
         old_EE_size = EE_idx.size();

    if(vertex_half_plane_trajectory_filter)
    {
        merge(PH_idx,
              PH_lambda,
              PH_cnt,
              vertex_half_plane_trajectory_filter->candidate_PHs(),
              vertex_half_plane_trajectory_filter->toi_PHs());
    }

    if(simplex_trajectory_filter)
    {
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
    }

    logger::info("Active set update: {} + {} + {} -> {} + {} + {}",
                 old_PH_size,
                 old_PT_size,
                 old_EE_size,
                 PH_idx.size(),
                 PT_idx.size(),
                 EE_idx.size());
}

void GlobalActiveSetManager::Impl::linearize_constraints()
{
    using namespace cuda_tool;
    auto thicknesses = global_vertex_manager->thicknesses();
    auto d_hats      = global_vertex_manager->d_hats();
    auto x           = non_penetrate_positions;
    auto vs          = global_simplicial_surface_manager->surf_vertices();
    auto edges       = global_simplicial_surface_manager->surf_edges();
    auto tris        = global_simplicial_surface_manager->surf_triangles();

    loose_resize(PHs, PH_idx.size());
    loose_resize(PH_d0, PH_idx.size());
    loose_resize(PH_d_grad, PH_idx.size());

    loose_resize(PTs, PT_idx.size());
    loose_resize(PT_d0, PT_idx.size());
    loose_resize(PT_d_grad, PT_idx.size());

    loose_resize(EEs, EE_idx.size());
    loose_resize(EE_d0, EE_idx.size());
    loose_resize(EE_d_grad, EE_idx.size());

    if(vertex_half_plane_trajectory_filter)
    {
        int n_ph = static_cast<int>(PH_idx.size());
        if(n_ph > 0)
            linearize_constraints_k1_kernel<<<
                cuda_tool::best_grid_dim(n_ph, linearize_constraints_k1_kernel),
                cuda_tool::best_block_dim(linearize_constraints_k1_kernel),
                0,
                nullptr>>>(thicknesses,
                           d_hats,
                           PH_idx.cview(),
                           x.cview(),
                           half_plane->positions(),
                           half_plane->normals(),
                           PHs.view(),
                           PH_d0.view(),
                           PH_d_grad.view(),
                           n_ph);
    }

    int n_pt = static_cast<int>(PT_idx.size());
    if(n_pt > 0)
        linearize_constraints_k2_kernel<<<
            cuda_tool::best_grid_dim(n_pt, linearize_constraints_k2_kernel),
            cuda_tool::best_block_dim(linearize_constraints_k2_kernel),
            0,
            nullptr>>>(thicknesses,
                       d_hats,
                       PT_idx.cview(),
                       vs,
                       tris,
                       x.cview(),
                       PTs.view(),
                       PT_d0.view(),
                       PT_d_grad.view(),
                       n_pt);

    int n_ee = static_cast<int>(EE_idx.size());
    if(n_ee > 0)
        linearize_constraints_k3_kernel<<<
            cuda_tool::best_grid_dim(n_ee, linearize_constraints_k3_kernel),
            cuda_tool::best_block_dim(linearize_constraints_k3_kernel),
            0,
            nullptr>>>(thicknesses,
                       d_hats,
                       EE_idx.cview(),
                       edges,
                       x.cview(),
                       EEs.view(),
                       EE_d0.view(),
                       EE_d_grad.view(),
                       EE_lambda.view(),
                       EE_cnt.view(),
                       1 << 30,
                       n_ee);
}

void GlobalActiveSetManager::Impl::update_slack()
{
    using namespace cuda_tool;
    auto x_hat = global_vertex_manager->positions();

    loose_resize(PH_slack, PHs.size());
    loose_resize(PT_slack, PTs.size());
    loose_resize(EE_slack, EEs.size());

    if(vertex_half_plane_trajectory_filter)
    {
        int n_ph = static_cast<int>(PHs.size());
        if(n_ph > 0)
            update_slack_k1_kernel<<<cuda_tool::best_grid_dim(n_ph, update_slack_k1_kernel),
                                     cuda_tool::best_block_dim(update_slack_k1_kernel),
                                     0,
                                     nullptr>>>(mu_vertices.cview(),
                                                PHs.cview(),
                                                x_hat,
                                                PH_d_grad.cview(),
                                                PH_lambda.cview(),
                                                PH_d0.view(),
                                                PH_slack.view(),
                                                n_ph);
    }

    int n_pt = static_cast<int>(PTs.size());
    if(n_pt > 0)
        update_slack_k2_kernel<<<cuda_tool::best_grid_dim(n_pt, update_slack_k2_kernel),
                                 cuda_tool::best_block_dim(update_slack_k2_kernel),
                                 0,
                                 nullptr>>>(mu_vertices.cview(),
                                            PTs.cview(),
                                            x_hat,
                                            PT_d_grad.cview(),
                                            PT_lambda.cview(),
                                            PT_d0.view(),
                                            PT_slack.view(),
                                            n_pt);

    int n_ee = static_cast<int>(EEs.size());
    if(n_ee > 0)
        update_slack_k3_kernel<<<cuda_tool::best_grid_dim(n_ee, update_slack_k3_kernel),
                                 cuda_tool::best_block_dim(update_slack_k3_kernel),
                                 0,
                                 nullptr>>>(mu_vertices.cview(),
                                            EEs.cview(),
                                            x_hat,
                                            EE_d_grad.cview(),
                                            EE_lambda.cview(),
                                            EE_d0.view(),
                                            EE_slack.view(),
                                            n_ee);
}

void GlobalActiveSetManager::Impl::update_lambda()
{
    using namespace cuda_tool;
    auto x_hat = global_vertex_manager->positions();

    if(vertex_half_plane_trajectory_filter)
    {
        int n_ph = static_cast<int>(PHs.size());
        if(n_ph > 0)
            update_lambda_k1_kernel<<<cuda_tool::best_grid_dim(n_ph, update_lambda_k1_kernel),
                                      cuda_tool::best_block_dim(update_lambda_k1_kernel),
                                      0,
                                      nullptr>>>(mu_vertices.cview(),
                                                 PHs.cview(),
                                                 x_hat,
                                                 PH_d_grad.cview(),
                                                 PH_d0.cview(),
                                                 PH_slack.cview(),
                                                 PH_lambda.view(),
                                                 PH_cnt.view(),
                                                 n_ph);
    }

    int n_pt = static_cast<int>(PTs.size());
    if(n_pt > 0)
        update_lambda_k2_kernel<<<cuda_tool::best_grid_dim(n_pt, update_lambda_k2_kernel),
                                  cuda_tool::best_block_dim(update_lambda_k2_kernel),
                                  0,
                                  nullptr>>>(mu_vertices.cview(),
                                             PTs.cview(),
                                             x_hat,
                                             PT_d_grad.cview(),
                                             PT_d0.cview(),
                                             PT_slack.cview(),
                                             PT_lambda.view(),
                                             PT_cnt.view(),
                                             n_pt);

    int n_ee = static_cast<int>(EEs.size());
    if(n_ee > 0)
        update_lambda_k3_kernel<<<cuda_tool::best_grid_dim(n_ee, update_lambda_k3_kernel),
                                  cuda_tool::best_block_dim(update_lambda_k3_kernel),
                                  0,
                                  nullptr>>>(mu_vertices.cview(),
                                             EEs.cview(),
                                             x_hat,
                                             EE_d_grad.cview(),
                                             EE_d0.cview(),
                                             EE_slack.cview(),
                                             EE_lambda.view(),
                                             EE_cnt.view(),
                                             n_ee);
}

void GlobalActiveSetManager::Impl::update_friction()
{
    PTs_friction.resize(PTs.size());
    PT_lambda_friction.resize(PTs.size());
    cuda_tool::BufferLaunch().copy<Vector4i>(PTs_friction.view(), std::as_const(PTs));
    cuda_tool::BufferLaunch().copy<Float>(PT_lambda_friction.view(), std::as_const(PT_lambda));

    EEs_friction.resize(EEs.size());
    EE_lambda_friction.resize(EEs.size());
    cuda_tool::BufferLaunch().copy<Vector4i>(EEs_friction.view(), std::as_const(EEs));
    cuda_tool::BufferLaunch().copy<Float>(EE_lambda_friction.view(), std::as_const(EE_lambda));

    PHs_friction.resize(PHs.size());
    PH_lambda_friction.resize(PHs.size());
    cuda_tool::BufferLaunch().copy<Vector2i>(PHs_friction.view(), std::as_const(PH_idx));
    cuda_tool::BufferLaunch().copy<Float>(PH_lambda_friction.view(), std::as_const(PH_lambda));
}

void GlobalActiveSetManager::Impl::record_non_penetrate_positions()
{
    auto x_hat = global_vertex_manager->positions();
    if(non_penetrate_positions.size() != x_hat.size())
        non_penetrate_positions.resize(x_hat.size());
    cuda_tool::BufferLaunch().copy<Vector3>(non_penetrate_positions.view(), std::as_const(x_hat));
    for(auto&& [i, R] : enumerate(active_set_reporters.view()))
    {
        R->record_non_penetrate_state();
    }
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
    global_vertex_manager->overwrite_positions(non_penetrate_positions.view());
}

void GlobalActiveSetManager::Impl::prepare_ccd()
{
    global_vertex_manager->setup_ccd(non_penetrate_positions.view());
}

void GlobalActiveSetManager::Impl::post_ccd()
{
    global_vertex_manager->restore_ccd();
}

void GlobalActiveSetManager::Impl::advance_non_penetrate_positions(Float alpha)
{
    auto x_hat = global_vertex_manager->positions();
    int  n     = static_cast<int>(non_penetrate_positions.size());
    if(n > 0)
        advance_non_penetrate_positions_kernel<<<
            cuda_tool::best_grid_dim(n, advance_non_penetrate_positions_kernel),
            cuda_tool::best_block_dim(advance_non_penetrate_positions_kernel),
            0,
            nullptr>>>(non_penetrate_positions.view(), x_hat, alpha, n);
    for(auto&& [i, R] : enumerate(active_set_reporters.view()))
    {
        R->advance_non_penetrate_state(alpha);
    }
}

cuda_tool::CBufferView<int> GlobalActiveSetManager::PHs() const
{
    return m_impl.PHs.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::PH_d0() const
{
    return m_impl.PH_d0.view();
}

cuda_tool::CBufferView<Vector3> GlobalActiveSetManager::PH_d_grad() const
{
    return m_impl.PH_d_grad.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::PH_lambda() const
{
    return m_impl.PH_lambda.view();
}

cuda_tool::CBufferView<int> GlobalActiveSetManager::PH_cnt() const
{
    return m_impl.PH_cnt.view();
}

cuda_tool::CBufferView<Vector2i> GlobalActiveSetManager::PHs_friction() const
{
    return m_impl.PHs_friction.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::PH_lambda_friction() const
{
    return m_impl.PH_lambda_friction.view();
}

cuda_tool::CBufferView<Vector4i> GlobalActiveSetManager::PTs() const
{
    return m_impl.PTs.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::PT_d0() const
{
    return m_impl.PT_d0.view();
}

cuda_tool::CBufferView<Vector12> GlobalActiveSetManager::PT_d_grad() const
{
    return m_impl.PT_d_grad.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::PT_lambda() const
{
    return m_impl.PT_lambda.view();
}

cuda_tool::CBufferView<int> GlobalActiveSetManager::PT_cnt() const
{
    return m_impl.PT_cnt.view();
}

cuda_tool::CBufferView<Vector4i> GlobalActiveSetManager::PTs_friction() const
{
    return m_impl.PTs_friction.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::PT_lambda_friction() const
{
    return m_impl.PT_lambda_friction.view();
}

cuda_tool::CBufferView<Vector4i> GlobalActiveSetManager::EEs() const
{
    return m_impl.EEs.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::EE_d0() const
{
    return m_impl.EE_d0.view();
}

cuda_tool::CBufferView<Vector12> GlobalActiveSetManager::EE_d_grad() const
{
    return m_impl.EE_d_grad.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::EE_lambda() const
{
    return m_impl.EE_lambda.view();
}

cuda_tool::CBufferView<int> GlobalActiveSetManager::EE_cnt() const
{
    return m_impl.EE_cnt.view();
}

cuda_tool::CBufferView<Vector4i> GlobalActiveSetManager::EEs_friction() const
{
    return m_impl.EEs_friction.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::EE_lambda_friction() const
{
    return m_impl.EE_lambda_friction.view();
}

cuda_tool::CBufferView<Vector3> GlobalActiveSetManager::non_penetrate_positions() const
{
    return m_impl.non_penetrate_positions.view();
}

cuda_tool::CBufferView<Float> GlobalActiveSetManager::mu_vertices() const
{
    return m_impl.mu_vertices.view();
}

Float GlobalActiveSetManager::decay_factor() const
{
    return m_impl.decay_factor;
}

Float GlobalActiveSetManager::toi_threshold() const
{
    return m_impl.toi_threshold;
}

Float GlobalActiveSetManager::alpha_lower_bound() const
{
    return m_impl.alpha_lower_bound;
}

GlobalActiveSetManager::NonPenetratePositionInfo::NonPenetratePositionInfo(Impl* impl,
                                                                           SizeT offset,
                                                                           SizeT count) noexcept
    : m_impl(impl)
    , m_offset(offset)
    , m_count(count)
{
}

cuda_tool::BufferView<Vector3> GlobalActiveSetManager::NonPenetratePositionInfo::non_penetrate_positions() const noexcept
{
    return m_impl->non_penetrate_positions.view(m_offset, m_count);
}

GlobalActiveSetManager::StiffnessEstimateInfo::StiffnessEstimateInfo(Impl* impl) noexcept
    : m_impl(impl)
{
}

cuda_tool::BufferView<Float> GlobalActiveSetManager::StiffnessEstimateInfo::mu_vertices(
    SizeT offset, SizeT count) const noexcept
{
    return m_impl->mu_vertices.view(offset, count);
}

Float GlobalActiveSetManager::StiffnessEstimateInfo::dt() const noexcept
{
    return m_impl->dt_attr->view()[0];
}

void GlobalActiveSetManager::Impl::init(WorldVisitor& world)
{
    auto config = world.scene().config();
    dt_attr     = config.find<Float>("dt");
    UIPC_ASSERT(dt_attr, "Scene config must have a 'dt' attribute.");
    decay_factor = config.find<Float>("contact/al-ipc/decay_factor")->view()[0];
    toi_threshold = config.find<Float>("contact/al-ipc/toi_threshold")->view()[0];
    alpha_lower_bound =
        config.find<Float>("contact/al-ipc/alpha_lower_bound")->view()[0];
    energy_enabled = true;
}

void GlobalActiveSetManager::init()
{
    m_impl.init(world());
}

void GlobalActiveSetManager::init_mu()
{
    m_impl.init_mu();
}

void GlobalActiveSetManager::filter_active()
{
    m_impl.filter_active();
}

void GlobalActiveSetManager::update_active_set()
{
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
    m_impl.update_lambda();
}

void GlobalActiveSetManager::update_friction()
{
    m_impl.update_friction();
}

void GlobalActiveSetManager::Impl::clear_friction_candidates()
{
    PTs_friction.resize(0);
    PT_lambda_friction.resize(0);
    EEs_friction.resize(0);
    EE_lambda_friction.resize(0);
    PHs_friction.resize(0);
    PH_lambda_friction.resize(0);
}

void GlobalActiveSetManager::Impl::snapshot_friction_candidates()
{
    if(should_discard_friction_candidates)
    {
        clear_friction_candidates();
        should_discard_friction_candidates = false;
        return;
    }
    linearize_constraints();
    update_friction();
}

void GlobalActiveSetManager::clear_friction_candidates()
{
    m_impl.clear_friction_candidates();
}

void GlobalActiveSetManager::snapshot_friction_candidates()
{
    m_impl.snapshot_friction_candidates();
}

void GlobalActiveSetManager::require_discard_friction()
{
    m_impl.should_discard_friction_candidates = true;
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

void GlobalActiveSetManager::prepare_ccd()
{
    m_impl.prepare_ccd();
}

void GlobalActiveSetManager::post_ccd()
{
    m_impl.post_ccd();
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

void GlobalActiveSetManager::add_reporter(ActiveSetReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_impl.active_set_reporters.register_sim_system(*reporter);
}

void GlobalActiveSetManager::add_stiffness_estimator(ALStiffnessEstimator* estimator)
{
    check_state(SimEngineState::BuildSystems, "add_stiffness_estimator()");
    m_impl.stiffness_estimators.register_sim_system(*estimator);
}
}  // namespace uipc::backend::cuda
