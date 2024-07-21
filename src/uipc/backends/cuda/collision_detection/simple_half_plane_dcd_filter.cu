#include <collision_detection/simple_half_plane_dcd_filter.h>
#include <muda/atomic.h>
#include <kernel_cout.h>
namespace uipc::backend::cuda
{
constexpr bool PrintDebugInfo = false;

REGISTER_SIM_SYSTEM(SimpleHalfPlaneDCDFilter);

void SimpleHalfPlaneDCDFilter::do_detect(FilterInfo& info)
{
    m_impl.detect(info);
}

void SimpleHalfPlaneDCDFilter::Impl::detect(FilterInfo& info)
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

muda::CBufferView<Vector2i> SimpleHalfPlaneDCDFilter::PHs() const
{
    return m_impl.PHs.view(0, m_impl.h_num_collisions);
}
}  // namespace uipc::backend::cuda
