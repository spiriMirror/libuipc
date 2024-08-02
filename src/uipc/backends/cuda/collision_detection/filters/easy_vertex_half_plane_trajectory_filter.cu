#include <collision_detection/filters/easy_vertex_half_plane_trajectory_filter.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(EasyVertexHalfPlaneTrajectoryFilter);

constexpr bool PrintDebugInfo = false;

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
    using namespace muda;

    auto query = [&]
    {
        num_collisions = 0;
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(info.surf_vertices().size(),
                   [num = num_collisions.viewer().name("num_collisions"),
                    surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                    positions = info.positions().viewer().name("positions"),
                    half_plane_positions = info.plane_positions().viewer().name("plane_positions"),
                    half_plane_normals = info.plane_normals().viewer().name("plane_normals"),
                    d_hat     = info.d_hat(),
                    PHs       = PHs.viewer().name("PHs"),
                    max_count = PHs.size()] __device__(int i) mutable
                   {
                       IndexT  vI  = surf_vertices(i);
                       Vector3 pos = positions(vI);

                       for(int j = 0; j < half_plane_positions.total_size(); ++j)
                       {
                           Vector3 plane_pos    = half_plane_positions(j);
                           Vector3 plane_normal = half_plane_normals(j);
                           Vector3 diff         = pos - plane_pos;

                           Float dst = diff.dot(plane_normal);

                           MUDA_ASSERT(dst > 0.0f, "dst=%f, why?", dst);

                           if(dst < d_hat)
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
    using namespace muda;

    info.toi().fill(1.1f);
    tois.resize(info.surf_vertices().size());

    // TODO: just hard code the slackness for now
    constexpr Float eta = 0.1;
    //constexpr Float slackness     = 0.8;
    //constexpr Float inv_slackness = 1.0 / slackness;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.surf_vertices().size(),
               [surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                positions = info.positions().viewer().name("positions"),
                displacements = info.displacements().viewer().name("displacements"),
                half_plane_positions = info.plane_positions().viewer().name("plane_positions"),
                half_plane_normals = info.plane_normals().viewer().name("plane_normals"),
                tois  = tois.viewer().name("tois"),
                alpha = info.alpha(),
                d_hat = info.d_hat(),
                eta] __device__(int i) mutable
               {
                   IndexT  vI  = surf_vertices(i);
                   Vector3 x   = positions(vI);
                   Vector3 dx  = displacements(vI) * alpha;
                   Vector3 x_t = x + dx;

                   Float min_toi = 1.1f;

                   for(int j = 0; j < half_plane_positions.total_size(); ++j)
                   {
                       Vector3 P = half_plane_positions(j);
                       Vector3 N = half_plane_normals(j);

                       Float t = N.dot(dx);
                       if(t >= 0)  // moving away from the plane, no collision
                           continue;

                       // t < 0, moving towards the plane

                       Vector3 diff = P - x;
                       Float t0 = N.dot(diff) * (1.0 - eta);  // gap should be larger than (eta * t0)

                       Float this_toi = t0 / t;

                       MUDA_ASSERT(this_toi > 0, "this_toi=%f, why?", this_toi);

                       min_toi = std::min(min_toi, this_toi);

                       if constexpr(PrintDebugInfo)
                       {
                           if(this_toi < 1.0)
                           {
                               cout << "vI: " << vI << ", pI: " << j
                                    << ", toi: " << this_toi << " d0: " << -t0 << "\n";
                           }
                       }
                   }

                   tois(i) = min_toi;
               });

    DeviceReduce().Min(tois.data(), info.toi().data(), info.surf_vertices().size());
}
}  // namespace uipc::backend::cuda
