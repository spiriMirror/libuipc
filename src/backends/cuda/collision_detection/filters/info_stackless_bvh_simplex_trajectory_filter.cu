#include <collision_detection/filters/info_stackless_bvh_simplex_trajectory_filter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(InfoStacklessBVHSimplexTrajectoryFilter);

void InfoStacklessBVHSimplexTrajectoryFilter::do_build(BuildInfo& info)
{
    (void)info;
    auto& config = world().scene().config();
    auto  method = config.find<std::string>("collision_detection/method");
    if(method->view()[0] != "info_stackless_bvh")
    {
        throw SimSystemException("Info stackless BVH unused");
    }
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_detect(DetectInfo& info)
{
    m_impl.detect(info);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_filter_active(FilterActiveInfo& info)
{
    m_impl.filter_active(info);
}

void InfoStacklessBVHSimplexTrajectoryFilter::do_filter_toi(FilterTOIInfo& info)
{
    m_impl.filter_toi(info);
}

muda::CBufferView<Vector2i>
InfoStacklessBVHSimplexTrajectoryFilter::candidate_PTs() const noexcept
{
    return m_impl.candidate_AllP_AllT_pairs.view();
}

muda::CBufferView<Vector2i>
InfoStacklessBVHSimplexTrajectoryFilter::candidate_EEs() const noexcept
{
    return m_impl.candidate_AllE_AllE_pairs.view();
}

muda::CBufferView<Float> InfoStacklessBVHSimplexTrajectoryFilter::toi_PTs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    return m_impl.tois.view(pp_size + pe_size, pt_size);
}

muda::CBufferView<Float> InfoStacklessBVHSimplexTrajectoryFilter::toi_EEs() const noexcept
{
    auto pp_size = m_impl.candidate_AllP_CodimP_pairs.size();
    auto pe_size = m_impl.candidate_CodimP_AllE_pairs.size();
    auto pt_size = m_impl.candidate_AllP_AllT_pairs.size();
    auto ee_size = m_impl.candidate_AllE_AllE_pairs.size();
    return m_impl.tois.view(pp_size + pe_size + pt_size, ee_size);
}
}  // namespace uipc::backend::cuda
