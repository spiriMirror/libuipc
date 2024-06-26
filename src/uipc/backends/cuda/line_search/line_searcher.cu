#include <line_search/line_searcher.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <line_search/line_search_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LineSearcher);

void LineSearcher::add_reporter(LineSearchReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    reporter->m_index = m_reporter_buffer.size();
    m_reporter_buffer.push_back(reporter);
}

void LineSearcher::do_build()
{
    on_init_scene([this]() { init(); });
}

void LineSearcher::init()
{
    auto scene = world().scene();

    m_reporters.resize(m_reporter_buffer.size());
    std::ranges::move(m_reporter_buffer, m_reporters.begin());

    m_energy_values.resize(m_reporters.size(), 0);

    m_report_energy = scene.info()["debug"]["report_energy"];
    m_dt            = scene.info()["dt"];
}

void LineSearcher::record_start_point()
{
    for(auto&& R : m_reporters)
    {
        RecordInfo info;
        R->record_start_point(info);
    }
}

void LineSearcher::step_forward(Float alpha)
{
    for(auto&& R : m_reporters)
    {
        StepInfo info;
        info.alpha = alpha;
        R->step_forward(info);
    }
}

Float LineSearcher::compute_energy()
{
    for(auto&& [i, action] : enumerate(m_reporters))
    {
        ComputeEnergyInfo info{this};
        action->compute_energy(info);
        UIPC_ASSERT(info.m_energy.has_value(),
                    "Energy not set by reporter, did you forget to call energy()?");
        m_energy_values[i] = info.m_energy.value();
    }

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
        for(auto&& [R, value] : zip(m_reporters, m_energy_values))
        {
            m_report_stream << "  * " << R->name() << "=" << value << "\n";
        }
        m_report_stream << "-------------------------------------------------------------------------------";
        spdlog::info(m_report_stream.str());
        m_report_stream.str("");
    }

    return total_energy;
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
}  // namespace uipc::backend::cuda
