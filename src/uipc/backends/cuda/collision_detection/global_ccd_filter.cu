#include <collision_detection/global_ccd_filter.h>
#include <collision_detection/simplex_ccd_filter.h>
#include <uipc/common/enumerate.h>
namespace uipc::backend::cuda
{
void GlobalCCDFilter::add_filter(CCDFilter* filter)
{
    check_state(SimEngineState::BuildSystems, "add_filter()");
    UIPC_ASSERT(filter != nullptr, "Input filter is nullptr");
    m_impl.filters.push_back(filter);
}

void GlobalCCDFilter::do_build()
{
    on_init_scene([this] { m_impl.build(); });
}

Float GlobalCCDFilter::filter_toi(Float alpha)
{
    return m_impl.filter_toi(alpha);
}

void GlobalCCDFilter::Impl::build()
{
    filters.reserve(filter_buffer.size());
    std::ranges::move(filter_buffer, std::back_inserter(filters));

    tois.resize(filters.size());
    h_tois.resize(filters.size());
}
Float GlobalCCDFilter::Impl::filter_toi(Float alpha)
{
    for(auto&& [i, filter] : enumerate(filters))
    {
        FilterInfo info;
        info.m_toi   = muda::VarView{tois.data() + i};
        info.m_alpha = alpha;
        filter->filter_toi(info);
    }
    tois.view().copy_to(h_tois.data());

    if constexpr(uipc::RUNTIME_CHECK)
    {
        for(auto&& [i, toi] : enumerate(h_tois))
        {
            UIPC_ASSERT(toi > 0.0f, "Invalid toi[{}] value: {}", filters[i]->name(), toi);
        }
    }

    return *std::min_element(h_tois.begin(), h_tois.end());
}
}  // namespace uipc::backend::cuda
