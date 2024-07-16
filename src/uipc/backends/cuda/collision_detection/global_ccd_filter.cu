#include <collision_detection/global_ccd_filter.h>
#include <collision_detection/simplex_ccd_filter.h>
#include <uipc/common/enumerate.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalCCDFilter);

void GlobalCCDFilter::add_filter(CCDFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input simplex_dcd_filter is nullptr");
    m_impl.filters.register_subsystem(*filter);
}

void GlobalCCDFilter::do_build()
{
    if(!world().scene().info()["contact"]["enable"])
    {
        throw SimSystemException("GlobalCCDFilter requires contact detection to be enabled");
    }

    on_init_scene([this] { m_impl.init(); });
}

Float GlobalCCDFilter::filter_toi(Float alpha)
{
    return m_impl.filter_toi(alpha);
}

void GlobalCCDFilter::Impl::init()
{
    filters.init();
    auto filter_view = filters.view();
    tois.resize(filter_view.size());
    h_tois.resize(filter_view.size());
}
Float GlobalCCDFilter::Impl::filter_toi(Float alpha)
{
    auto filter_view = filters.view();
    for(auto&& [i, filter] : enumerate(filter_view))
    {
        FilterInfo info;
        info.m_toi   = muda::VarView<Float>{tois.data() + i};
        info.m_alpha = alpha;
        filter->filter_toi(info);
    }
    tois.view().copy_to(h_tois.data());

    if constexpr(uipc::RUNTIME_CHECK)
    {
        for(auto&& [i, toi] : enumerate(h_tois))
        {
            UIPC_ASSERT(toi > 0.0f, "Invalid toi[{}] value: {}", filter_view[i]->name(), toi);
        }
    }

    auto min_toi = *std::min_element(h_tois.begin(), h_tois.end());

    return min_toi < 1.0 ? min_toi : 1.0;
}
}  // namespace uipc::backend::cuda
