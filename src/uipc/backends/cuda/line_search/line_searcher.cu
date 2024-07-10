#include <line_search/line_searcher.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>
#include <line_search/line_search_reporter.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LineSearcher);

void LineSearcher::do_build()
{
    on_init_scene([this]() { init(); });
}

void LineSearcher::init()
{
    auto scene = world().scene();

    m_reporters.build();

    m_energy_values.resize(m_reporters.view().size() + m_energy_reporters.size(), 0);

    for(auto&& [i, R] : enumerate(m_reporters.view()))
        R->m_index = i;


    m_report_energy = scene.info()["line_search"]["report_energy"];
    m_max_iter      = scene.info()["line_search"]["max_iter"];
    m_dt            = scene.info()["dt"];
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

Float LineSearcher::compute_energy()
{
    auto reporter_energyes = span{m_energy_values}.subspan(0, m_reporters.view().size());

    for(auto&& [E, R] : zip(reporter_energyes, m_reporters.view()))
    {
        EnergyInfo info{this};
        R->compute_energy(info);
        UIPC_ASSERT(info.m_energy.has_value(),
                    "Energy[{}] not set by reporter, did you forget to call energy()?",
                    R->name());
        E = info.m_energy.value();
        UIPC_ASSERT(!std::isnan(E) && std::isfinite(E), "Energy [{}] is {}", R->name(), E);
    }

    auto energy_reporter_energyes =
        span{m_energy_values}.subspan(m_reporters.view().size());

    for(auto&& [E, ER, name] :
        zip(energy_reporter_energyes, m_energy_reporters, m_energy_reporter_names))
    {
        EnergyInfo info{this};
        ER(info);
        UIPC_ASSERT(info.m_energy.has_value(),
                    "Energy[{}] not set by reporter, did you forget to call energy()?",
                    name);
        E = info.m_energy.value();
        UIPC_ASSERT(!std::isnan(E) && std::isfinite(E), "Energy [{}] is {}", name, E);
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
        for(auto&& [R, value] : zip(m_reporters, reporter_energyes))
        {
            m_report_stream << "  > " << R->name() << "=" << value << "\n";
        }

        for(auto&& [ER, value, name] :
            zip(m_energy_reporters, energy_reporter_energyes, m_energy_reporter_names))
        {
            m_report_stream << "  * " << name << "=" << value << "\n";
        }

        m_report_stream << "-------------------------------------------------------------------------------";
        spdlog::info(m_report_stream.str());
        m_report_stream.str("");
    }

    return total_energy;
}

void LineSearcher::add_reporter(LineSearchReporter* reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_reporters.register_subsystem(reporter);
}

void LineSearcher::add_reporter(std::string_view energy_name,
                                std::function<void(EnergyInfo)>&& energy_reporter)
{
    check_state(SimEngineState::BuildSystems, "add_reporter()");
    m_energy_reporters.push_back(std::move(energy_reporter));
    m_energy_reporter_names.push_back(std::string{energy_name});
}

LineSearcher::EnergyInfo::EnergyInfo(LineSearcher* impl) noexcept
    : m_impl(impl)
{
}

Float LineSearcher::EnergyInfo::dt() noexcept
{
    return m_impl->m_dt;
}

void LineSearcher::EnergyInfo::energy(Float e) noexcept
{
    m_energy = e;
}

SizeT LineSearcher::max_iter() const noexcept
{
    return m_max_iter;
}

}  // namespace uipc::backend::cuda
