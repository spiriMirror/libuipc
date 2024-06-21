#include <line_searcher.h>
#include <uipc/common/enumerate.h>
#include <uipc/common/zip.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(LineSearcher);

void LineSearcher::build()
{
    on_init_scene([this]() { init(); });
}


void LineSearcher::on_record_current_state(std::function<void()>&& action)
{
    check_state(SimEngineState::BuildSystems, "on_record_current_state()");
    m_record_current_state.push_back(std::move(action));
}

void LineSearcher::on_step_forward(std::function<void(const StepInfo& info)>&& action)
{
    check_state(SimEngineState::BuildSystems, "on_step_forward()");
    m_step_forwards.push_back(std::move(action));
}

void LineSearcher::on_compute_energy(std::string_view name,
                                     std::function<Float(const ComputeEnergyInfo& info)>&& action)
{
    check_state(SimEngineState::BuildSystems, "on_compute_energy()");
    m_compute_energy.push_back(std::move(action));
    m_energy_names.push_back(std::string{name});
    m_energy_values.push_back(0.0);
}

void LineSearcher::init()
{
    m_report_energy = world().scene().info()["debug"]["report_energy"];
    m_energy_values.resize(m_compute_energy.size(), 0);
}

void LineSearcher::record_current_state()
{
    for(auto& action : m_record_current_state)
    {
        action();
    }
}

void LineSearcher::step_forward(Float alpha)
{
    StepInfo info;
    info.alpha = alpha;
    for(auto& action : m_step_forwards)
    {
        action(info);
    }
}

Float LineSearcher::compute_energy()
{
    ComputeEnergyInfo info;
    for(auto [i, action] : enumerate(m_compute_energy))
    {
        m_energy_values[i] = action(info);
    }

    Float total_energy =
        std::accumulate(m_energy_values.begin(), m_energy_values.end(), 0.0);

    if(m_report_energy)
    {
        report_stream << "Compute Energy: ";
        report_stream << "Total=" << total_energy << "\n";

        for(auto [name, value] : zip(m_energy_names, m_energy_values))
        {
            report_stream << name << "=" << value << "\n";
        }
        spdlog::info(report_stream.str());
    }

    return total_energy;
}
}  // namespace uipc::backend::cuda
