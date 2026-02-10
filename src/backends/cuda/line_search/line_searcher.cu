#include <line_search/line_searcher.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <line_search/line_search_reporter.h>
#include <uipc/common/timer.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LineSearcher);

void LineSearcher::do_build() {}

void LineSearcher::init()
{
    auto scene = world().scene();

    auto report_enery_attr = scene.config().find<IndexT>("line_search/report_energy");
    m_report_energy = report_enery_attr->view()[0] != 0;

    auto max_iter_attr = scene.config().find<IndexT>("line_search/max_iter");
    m_max_iter         = max_iter_attr->view()[0];

    auto dt_attr = scene.config().find<Float>("dt");
    m_dt         = dt_attr->view()[0];

    m_energy_values.resize(m_reporters.view().size(), 0);

    auto reporter_view = m_reporters.view();

    for(auto&& [i, R] : enumerate(reporter_view))
        R->m_index = i;

    for(auto&& [i, R] : enumerate(reporter_view))
        R->init();
}

void LineSearcher::record_start_point()
{
    for(auto&& R : m_reporters.view())
    {
        RecordInfo info;
        R->record_start_point(info);
    }
}

void LineSearcher::step_forward(Float alpha)
{
    for(auto&& R : m_reporters.view())
    {
        StepInfo info;
        info.alpha = alpha;
        R->step_forward(info);
    }
}

Float LineSearcher::compute_energy(bool is_initial)
{
    Timer timer{"Compute Energy"};

    auto reporter_energyes = span{m_energy_values}.subspan(0, m_reporters.view().size());

    for(auto&& [E, R] : zip(reporter_energyes, m_reporters.view()))
    {
        ComputeEnergyInfo info{this};
        info.m_is_initial = is_initial;
        R->compute_energy(info);
        UIPC_ASSERT(info.m_energy.has_value(),
                    "Energy[{}] not set by reporter, did you forget to call energy()?",
                    R->name());
        E = info.m_energy.value();
        UIPC_ASSERT(!std::isnan(E) && std::isfinite(E), "Energy [{}] is {}", R->name(), E);
    }

    auto energy_reporter_energyes =
        span{m_energy_values}.subspan(m_reporters.view().size());

    Float total_energy =
        std::accumulate(m_energy_values.begin(), m_energy_values.end(), 0.0);

    if(m_report_energy)
    {
        m_report_stream << R"(
-------------------------------------------------------------------------------
*                             Compute Energy                                  *
-------------------------------------------------------------------------------
)";
        m_report_stream << "Total:" << total_energy << "\n";
        for(auto&& [R, value] : zip(m_reporters.view(), reporter_energyes))
        {
            m_report_stream << "  > " << R->name() << "=" << value << "\n";
        }
        m_report_stream << "-------------------------------------------------------------------------------";
        logger::info(m_report_stream.str());
        m_report_stream.str("");
    }

    return total_energy;
}

void LineSearcher::add_reporter(LineSearchReporter* reporter)
{
    UIPC_ASSERT(reporter, "reporter is nullptr");
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_reporters.register_sim_system(*reporter);
}

LineSearcher::ComputeEnergyInfo::ComputeEnergyInfo(LineSearcher* impl) noexcept
    : m_impl(impl)
{
}

Float LineSearcher::ComputeEnergyInfo::dt() noexcept
{
    return m_impl->m_dt;
}

void LineSearcher::ComputeEnergyInfo::energy(Float e) noexcept
{
    m_energy = e;
}

bool LineSearcher::ComputeEnergyInfo::is_initial() noexcept
{
    return m_is_initial;
}

SizeT LineSearcher::max_iter() const noexcept
{
    return m_max_iter;
}
}  // namespace uipc::backend::cuda
