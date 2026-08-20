#include <collision_detection/filters/al_vertex_half_plane_trajectory_filter.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>
#include <utils/codim_thickness.h>
#include <pipeline/al_ipc_pipeline_flag.h>

namespace uipc::backend::cuda
{
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

    // TODO: just hard code the slackness for now
    constexpr Float eta = 0.01;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(
            info.surf_vertices().size(),
            [surf_vertices = info.surf_vertices().viewer(),
             plane_vertex_offset = info.half_plane_vertex_offset(),
             positions           = info.positions().viewer(),
             thicknesses = info.thicknesses().viewer(),
             contact_element_ids = info.contact_element_ids().viewer(),
             subscene_element_ids = info.subscene_element_ids().viewer(),
             subscene_mask_tabular = info.subscene_mask_tabular().viewer(),
             contact_mask_tabular = info.contact_mask_tabular().viewer(),
             displacements = info.displacements().viewer(),
             half_plane_positions = info.plane_positions().viewer(),
             half_plane_normals = info.plane_normals().viewer(),
             PHs   = PHs.viewer(),
             tois  = tois.viewer(),
             alpha = info.alpha(),
             eta] __device__(int i) mutable
            {
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
            });

    DeviceReduce().Min(tois.data(), info.toi().data(), tois.size());
}
}  // namespace uipc::backend::cuda
