#include <collision_detection/filters/al_vertex_half_plane_trajectory_filter.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>
#include <utils/codim_thickness.h>
#include <pipeline/al_ipc_pipeline_flag.h>

namespace uipc::backend::cuda
{
namespace
{
    // TODO: just hard code the slackness for now
    constexpr Float eta = 0.01;

    __global__ void ALVertexHalfPlaneTrajectoryFilter_filter_toi_kernel(
        cuda_tool::CBufferView<IndexT> surf_vertices,
        IndexT                         plane_vertex_offset,
        cuda_tool::CBufferView<Vector3> positions,
        cuda_tool::CBufferView<Float>  thicknesses,
        cuda_tool::CBufferView<IndexT> contact_element_ids,
        cuda_tool::CBufferView<IndexT> subscene_element_ids,
        cuda_tool::CDense2D<IndexT>    subscene_mask_tabular,
        cuda_tool::CDense2D<IndexT>    contact_mask_tabular,
        cuda_tool::CBufferView<Vector3> displacements,
        cuda_tool::CBufferView<Vector3> half_plane_positions,
        cuda_tool::CBufferView<Vector3> half_plane_normals,
        cuda_tool::BufferView<Vector2i> PHs,
        cuda_tool::BufferView<Float>   tois,
        Float                          alpha,
        int                            n)
    {
        int i = blockIdx.x * blockDim.x + threadIdx.x;
        if(i >= n)
            return;
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

            PHs(i * half_plane_positions.total_size() + j) = Vector2i{vI, j};
            if(t <= 0)
            {
                // moving away from the plane, no collision
                tois(i * half_plane_positions.total_size() + j) = 1.1;
            }
            else
            {
                // t > 0, moving towards the plane
                Vector3 diff = P - x;
                Float   t0   = -N.dot(diff) - thickness;

                Float this_toi = t0 / t * (1 - eta);

                tois(i * half_plane_positions.total_size() + j) = this_toi;
            }
        }
    }
}  // namespace

REGISTER_SIM_SYSTEM(ALVertexHalfPlaneTrajectoryFilter);

constexpr bool PrintDebugInfo = false;

void ALVertexHalfPlaneTrajectoryFilter::do_build(BuildInfo& info)
{
    require<ALIPCPipelineFlag>();
}

cuda_tool::CBufferView<Vector2i> ALVertexHalfPlaneTrajectoryFilter::candidate_PHs() const noexcept
{
    return m_impl.PHs;
}

cuda_tool::CBufferView<Float> ALVertexHalfPlaneTrajectoryFilter::toi_PHs() const noexcept
{
    return m_impl.tois;
}

void ALVertexHalfPlaneTrajectoryFilter::do_detect(DetectInfo& info)
{
    // do nothing
}

void ALVertexHalfPlaneTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void ALVertexHalfPlaneTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

void ALVertexHalfPlaneTrajectoryFilter::Impl::filter_active(FilterActiveInfo& info)
{
    // do nothing
}

void ALVertexHalfPlaneTrajectoryFilter::Impl::filter_toi(FilterTOIInfo& info)
{
    using namespace cuda_tool;

    info.toi().fill(1.1f);
    tois.resize(info.surf_vertices().size() * info.plane_positions().size());
    tois.fill(1.1f);
    PHs.resize(info.surf_vertices().size() * info.plane_positions().size());

    int n = static_cast<int>(info.surf_vertices().size());
    if(n > 0)
    {
        auto k = ALVertexHalfPlaneTrajectoryFilter_filter_toi_kernel;
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
            PHs.view(),
            tois.view(),
            info.alpha(),
            n);
    }

    DeviceReduce().Min(tois.data(), info.toi().data(), tois.size());
}
}  // namespace uipc::backend::cuda
