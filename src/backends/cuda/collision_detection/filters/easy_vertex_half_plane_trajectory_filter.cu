#include <collision_detection/filters/easy_vertex_half_plane_trajectory_filter.h>
#include <cuda_tool/cub.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>
#include <utils/codim_thickness.h>
#include <pipeline/ipc_pipeline_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    // TODO: just hard code the slackness for now
    constexpr Float eta = 0.1;

    __global__ void EasyVertexHalfPlaneTrajectoryFilter_filter_active_kernel(
        cuda_tool::Dense<IndexT>        num,
        IndexT                          plane_vertex_offset,
        cuda_tool::CBufferView<IndexT>  surf_vertices,
        cuda_tool::CBufferView<Vector3> positions,
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<IndexT>  contact_element_ids,
        cuda_tool::CBufferView<IndexT>  subscene_element_ids,
        cuda_tool::CDense2D<IndexT>     contact_mask_tabular,
        cuda_tool::CDense2D<IndexT>     subscene_mask_tabular,
        cuda_tool::CBufferView<Vector3> half_plane_positions,
        cuda_tool::CBufferView<Vector3> half_plane_normals,
        cuda_tool::BufferView<Vector2i> PHs,
        cuda_tool::CBufferView<Float>   d_hats,
        size_t                          max_count,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        for(int j = 0; j < half_plane_positions.total_size(); ++j)
        {
            IndexT vI = surf_vertices(i);
            IndexT vJ = plane_vertex_offset + j;

            Float d_hat = d_hats(vI);

            IndexT L = contact_element_ids(vI);
            IndexT R = contact_element_ids(vJ);

            IndexT sL = subscene_element_ids(vI);
            IndexT sR = subscene_element_ids(vJ);

            if(subscene_mask_tabular(sL, sR) == 0)
                continue;

            if(contact_mask_tabular(L, R) == 0)
                continue;

            Vector3 pos = positions(vI);

            Vector3 plane_pos    = half_plane_positions(j);
            Vector3 plane_normal = half_plane_normals(j);

            Vector3 diff = pos - plane_pos;

            Float dst = diff.dot(plane_normal);

            Float thickness = thicknesses(vI);

            Float D = dst * dst;

            auto range = D_range(thickness, d_hat);

            if(is_active_D(range, D))
            {
                auto last = cuda_tool::atomic_add(num.data(), 1);

                if(last < max_count)
                {
                    PHs(last) = Vector2i{vI, j};
                }
            }
        }
    }

    __global__ void EasyVertexHalfPlaneTrajectoryFilter_filter_toi_kernel(
        cuda_tool::CBufferView<IndexT>  surf_vertices,
        IndexT                          plane_vertex_offset,
        cuda_tool::CBufferView<Vector3> positions,
        cuda_tool::CBufferView<Float>   thicknesses,
        cuda_tool::CBufferView<IndexT>  contact_element_ids,
        cuda_tool::CBufferView<IndexT>  subscene_element_ids,
        cuda_tool::CDense2D<IndexT>     subscene_mask_tabular,
        cuda_tool::CDense2D<IndexT>     contact_mask_tabular,
        cuda_tool::CBufferView<Vector3> displacements,
        cuda_tool::CBufferView<Vector3> half_plane_positions,
        cuda_tool::CBufferView<Vector3> half_plane_normals,
        cuda_tool::BufferView<Float>    tois,
        Float                           alpha,
        int                             n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
        Float min_toi = 1.1f;  // large enough

        for(int j = 0; j < half_plane_positions.total_size(); ++j)
        {
            IndexT vI = surf_vertices(i);
            IndexT vJ = plane_vertex_offset + j;

            IndexT L = contact_element_ids(vI);
            IndexT R = contact_element_ids(vJ);

            IndexT sL = subscene_element_ids(vI);
            IndexT sR = subscene_element_ids(vJ);

            if(subscene_mask_tabular(sL, sR) == 0)
                continue;

            if(contact_mask_tabular(L, R) == 0)
                continue;

            Vector3 x   = positions(vI);
            Vector3 dx  = displacements(vI) * alpha;
            Vector3 x_t = x + dx;


            Vector3 P = half_plane_positions(j);
            Vector3 N = half_plane_normals(j);

            Float thickness = thicknesses(vI);

            Float t = -N.dot(dx);
            if(t <= 0)  // moving away from the plane, no collision
                continue;

            // t > 0, moving towards the plane


            Vector3 diff = P - x;
            Float   t0   = -N.dot(diff) - thickness;

            Float this_toi = t0 / t * (1 - eta);

            min_toi = std::min(min_toi, this_toi);
        }

        tois(i) = min_toi;
    }
}  // namespace

REGISTER_SIM_SYSTEM(EasyVertexHalfPlaneTrajectoryFilter);

constexpr bool PrintDebugInfo = false;

void EasyVertexHalfPlaneTrajectoryFilter::do_build(BuildInfo& info)
{
    require<IPCPipelineFlag>();
}

cuda_tool::CBufferView<Vector2i> EasyVertexHalfPlaneTrajectoryFilter::candidate_PHs() const noexcept
{
    return m_impl.PHs;
}

cuda_tool::CBufferView<Float> EasyVertexHalfPlaneTrajectoryFilter::toi_PHs() const noexcept
{
    return m_impl.tois;
}

void EasyVertexHalfPlaneTrajectoryFilter::do_detect(DetectInfo& info)
{
    // do nothing
}

void EasyVertexHalfPlaneTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void EasyVertexHalfPlaneTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

void EasyVertexHalfPlaneTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    auto query = [&]
    {
        num_collisions = 0;

        int n = static_cast<int>(info.surf_vertices().size());
        if(n > 0)
        {
            auto k = EasyVertexHalfPlaneTrajectoryFilter_filter_active_kernel;
            k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
                num_collisions.viewer(),
                info.half_plane_vertex_offset(),
                info.surf_vertices(),
                info.positions(),
                info.thicknesses(),
                info.contact_element_ids(),
                info.subscene_element_ids(),
                info.contact_mask_tabular().viewer(),
                info.subscene_mask_tabular().viewer(),
                info.plane_positions(),
                info.plane_normals(),
                PHs.view(),
                info.d_hats(),
                PHs.size(),
                n);
        }
    };

    query();

    h_num_collisions = num_collisions;

    if(h_num_collisions > PHs.size())
    {
        const auto new_size = static_cast<size_t>(h_num_collisions * reserve_ratio);
        PHs.reserve_discard(new_size);
        PHs.resize_discard(new_size);
        query();
    }

    info.PHs(PHs.view(0, h_num_collisions));

    if constexpr(PrintDebugInfo)
    {
        std::vector<Vector2i> phs(h_num_collisions);
        PHs.view(0, h_num_collisions).copy_to(phs.data());
        for(auto& ph : phs)
        {
            std::cout << "vI: " << ph[0] << ", pI: " << ph[1] << std::endl;
        }
    }
}

void EasyVertexHalfPlaneTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace cuda_tool;

    info.toi().fill(1.1f);
    tois.resize_discard(info.surf_vertices().size());

    int n = static_cast<int>(info.surf_vertices().size());
    if(n > 0)
    {
        auto k = EasyVertexHalfPlaneTrajectoryFilter_filter_toi_kernel;
        k<<<cuda_tool::best_grid_dim(n, k), cuda_tool::best_block_dim(k), 0, nullptr>>>(
            info.surf_vertices(),
            info.half_plane_vertex_offset(),
            info.positions(),
            info.thicknesses(),
            info.contact_element_ids(),
            info.subscene_element_ids(),
            info.subscene_mask_tabular().viewer(),
            info.contact_mask_tabular().viewer(),
            info.displacements(),
            info.plane_positions(),
            info.plane_normals(),
            tois.view(),
            info.alpha(),
            n);
    }

    DeviceReduce().Min(tois.data(), info.toi().data(), info.surf_vertices().size());
}
}  // namespace uipc::backend::cuda
