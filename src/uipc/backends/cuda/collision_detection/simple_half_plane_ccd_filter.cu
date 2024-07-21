#include <collision_detection/simple_half_plane_ccd_filter.h>
#include <muda/cub/device/device_reduce.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo = true;

REGISTER_SIM_SYSTEM(SimpleHalfPlaneCCDFilter);

void SimpleHalfPlaneCCDFilter::Impl::filter_toi(FilterInfo& info)
{
    using namespace muda;

    info.toi().fill(1.1f);
    tois.resize(info.surf_vertices().size());

    // TODO: just hard code the slackness for now
    constexpr Float slackness     = 0.8;
    constexpr Float inv_slackness = 1.0 / slackness;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.surf_vertices().size(),
               [surf_vertices = info.surf_vertices().viewer().name("surf_vertices"),
                positions = info.positions().viewer().name("positions"),
                displacements = info.displacements().viewer().name("displacements"),
                half_plane_positions = info.plane_positions().viewer().name("plane_positions"),
                half_plane_normals = info.plane_normals().viewer().name("plane_normals"),
                tois          = tois.viewer().name("tois"),
                alpha         = info.alpha(),
                d_hat         = info.d_hat(),
                slackness     = slackness,
                inv_slackness = inv_slackness] __device__(int i) mutable
               {
                   IndexT  vI       = surf_vertices(i);
                   Vector3 pos      = positions(vI);
                   Vector3 dx       = displacements(vI) * alpha;
                   Vector3 pos_next = pos + dx;

                   Float min_toi = 1.1f;

                   for(int j = 0; j < half_plane_positions.total_size(); ++j)
                   {
                       Vector3 plane_pos    = half_plane_positions(j);
                       Vector3 plane_normal = half_plane_normals(j);

                       Float t = plane_normal.dot(dx);
                       if(t >= 0)  // moving away from the plane, no collision
                           continue;

                       Vector3 diff = plane_pos - pos;
                       Float   t0   = plane_normal.dot(diff);

                       Float this_toi = t0 / t;

                       MUDA_ASSERT(this_toi > 0, "this_toi=%f, why?", this_toi);

                       if(this_toi <= inv_slackness)
                           this_toi *= slackness;

                       min_toi = std::min(min_toi, this_toi);

                       if constexpr(PrintDebugInfo)
                       {
                           if(this_toi < 1.0)
                           {
                               cout << "vI: " << vI << ", pI: " << j << ", toi: " << this_toi
                                    << "slackness: " << slackness << "\n";
                           }
                       }
                   }

                   tois(i) = min_toi;
               });

    DeviceReduce().Min(tois.data(), info.toi().data(), info.surf_vertices().size());
}

void SimpleHalfPlaneCCDFilter::do_filter_toi(FilterInfo& info)
{
    m_impl.filter_toi(info);
}
}  // namespace uipc::backend::cuda
