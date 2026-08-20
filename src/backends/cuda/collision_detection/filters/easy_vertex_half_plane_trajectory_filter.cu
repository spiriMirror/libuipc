#include <collision_detection/filters/easy_vertex_half_plane_trajectory_filter.h>
#include <cuda_tool/cuda_tool.h>
#include <kernel_cout.h>
#include <utils/codim_thickness.h>
#include <pipeline/ipc_pipeline_flag.h>

namespace uipc::backend::cuda
{
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
    using namespace cuda_tool;

    auto query = [&]
    {
        num_collisions = 0;

        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(info.surf_vertices().size(),
                   [num = num_collisions.viewer(),
                    plane_vertex_offset = info.half_plane_vertex_offset(),
                    surf_vertices = info.surf_vertices().viewer(),
                    positions = info.positions().viewer(),
                    thicknesses = info.thicknesses().viewer(),
                    contact_element_ids = info.contact_element_ids().viewer(),
                    subscene_element_ids = info.subscene_element_ids().viewer(),
                    contact_mask_tabular = info.contact_mask_tabular().viewer(),
                    subscene_mask_tabular =
                        info.subscene_mask_tabular().viewer(),
                    half_plane_positions = info.plane_positions().viewer(),
                    half_plane_normals = info.plane_normals().viewer(),
                    PHs       = PHs.viewer(),
                    d_hats    = info.d_hats().viewer(),
                    max_count = PHs.size()] __device__(int i) mutable
                   {
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
                               auto last = atomic_add(num.data(), 1);

                               if(last < max_count)
                               {
                                   PHs(last) = Vector2i{vI, j};
                               }
                           }
                       }
                   });
    };

    query();

    h_num_collisions = num_collisions;

    if(h_num_collisions > PHs.size())
    {
        PHs.resize(h_num_collisions * reserve_ratio);
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
    tois.resize(info.surf_vertices().size());

    // TODO: just hard code the slackness for now
    constexpr Float eta = 0.1;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(info.surf_vertices().size(),
               [surf_vertices = info.surf_vertices().viewer(),
                plane_vertex_offset = info.half_plane_vertex_offset(),
                positions   = info.positions().viewer(),
                thicknesses = info.thicknesses().viewer(),
                contact_element_ids = info.contact_element_ids().viewer(),
                subscene_element_ids = info.subscene_element_ids().viewer(),
                subscene_mask_tabular = info.subscene_mask_tabular().viewer(),
                contact_mask_tabular = info.contact_mask_tabular().viewer(),
                displacements = info.displacements().viewer(),
                half_plane_positions = info.plane_positions().viewer(),
                half_plane_normals = info.plane_normals().viewer(),
                tois  = tois.viewer(),
                alpha = info.alpha(),
                eta] __device__(int i) mutable
               {
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
               });

    DeviceReduce().Min(tois.data(), info.toi().data(), info.surf_vertices().size());
}
}  // namespace uipc::backend::cuda
