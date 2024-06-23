#include <global_linear_system.h>
#include <diag_linear_subsystem.h>
#include <off_diag_linear_subsystem.h>
#include <uipc/common/range.h>
namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalLinearSystem);

void GlobalLinearSystem::do_build()
{
    on_init_scene([this] { m_impl.init(); });
}

void GlobalLinearSystem::solve()
{
    m_impl.build_linear_system();
    m_impl.solve_linear_system();
    m_impl.distribute_solution();
}


void GlobalLinearSystem::Impl::init()
{
    diag_subsystems.resize(diag_subsystem_buffer.size());
    off_diag_subsystems.resize(off_diag_subsystem_buffer.size());

    std::ranges::move(diag_subsystem_buffer, diag_subsystems.begin());
    std::ranges::move(off_diag_subsystem_buffer, off_diag_subsystems.begin());

    auto total_count = diag_subsystems.size() + off_diag_subsystems.size();

    subsystem_infos.resize(total_count);

    auto diag_span = span{subsystem_infos}.subspan(0, diag_subsystems.size());
    auto off_diag_span = span{subsystem_infos}.subspan(diag_subsystems.size(),
                                                       off_diag_subsystems.size());

    auto offset = 0;
    for(auto i : range(diag_span.size()))
    {
        auto& dst_diag       = diag_span[i];
        dst_diag.is_diag     = true;
        dst_diag.local_index = i;
        dst_diag.index       = offset + i;
    }

    offset += diag_subsystems.size();
    for(auto i : range(off_diag_span.size()))
    {
        auto& dst_off_diag       = off_diag_span[i];
        dst_off_diag.is_diag     = false;
        dst_off_diag.local_index = i;
        dst_off_diag.index       = offset + i;
    }

    subsystem_triplet_offsets.resize(total_count, ~0ull);
    subsystem_triplet_offsets.resize(total_count, ~0ull);

    diag_dof_offsets.resize(diag_subsystems.size());
    diag_dof_counts.resize(diag_subsystems.size());
}

void GlobalLinearSystem::Impl::build_linear_system()
{
    _update_subsystem_extent();
}

void GlobalLinearSystem::Impl::_update_subsystem_extent()
{
    bool dof_count_changed     = false;
    bool triplet_count_changed = false;

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto           dof_i          = subsystem_info.local_index;
            auto           triplet_i      = subsystem_info.index;
            auto&          diag_subsystem = diag_subsystems[dof_i];
            DiagExtentInfo info;
            diag_subsystem->report_extent(info);
            UIPC_ASSERT(info.m_storage_type == HessianStorageType::Full,
                        "Only support full storage type for now");

            dof_count_changed |= diag_dof_counts[dof_i] != info.m_dof_count;
            diag_dof_counts[dof_i] = info.m_dof_count;

            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != info.m_block_count;
            subsystem_triplet_counts[triplet_i] = info.m_block_count;
        }
        else
        {
            auto triplet_i = subsystem_info.index;
            auto& off_diag_subsystem = off_diag_subsystems[subsystem_info.local_index];
            OffDiagExtentInfo info;
            off_diag_subsystem->report_extent(info);

            UIPC_ASSERT(info.m_storage_type == HessianStorageType::Full,
                        "Only support full storage type for now");

            auto total_block_count = info.m_lr_block_count + info.m_rl_block_count;

            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != total_block_count;
            subsystem_triplet_counts[triplet_i] = total_block_count;
        }
    }

    if(dof_count_changed)
    {
        std::exclusive_scan(
            diag_dof_counts.begin(), diag_dof_counts.end(), diag_dof_offsets.begin(), 0);
    }

    if(triplet_count_changed)
    {
        std::exclusive_scan(subsystem_triplet_counts.begin(),
                            subsystem_triplet_counts.end(),
                            subsystem_triplet_offsets.begin(),
                            0);
    }
}

void GlobalLinearSystem::Impl::solve_linear_system() {}
void GlobalLinearSystem::Impl::distribute_solution() {}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void GlobalLinearSystem::add_subsystem(DiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    m_impl.add_subsystem(subsystem);
}

void GlobalLinearSystem::add_subsystem(OffDiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    subsystem->check_dep_systems();
    m_impl.add_subsystem(subsystem);
}

void GlobalLinearSystem::Impl::add_subsystem(DiagLinearSubsystem* subsystem)
{
    diag_subsystem_buffer.push_back(subsystem);
}

void GlobalLinearSystem::Impl::add_subsystem(OffDiagLinearSubsystem* subsystem)
{
    off_diag_subsystem_buffer.push_back(subsystem);
}
}  // namespace uipc::backend::cuda
