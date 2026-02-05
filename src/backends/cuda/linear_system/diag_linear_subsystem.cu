#include <linear_system/diag_linear_subsystem.h>

namespace uipc::backend::cuda
{
U64 DiagLinearSubsystem::uid() const noexcept
{
    return get_uid();
}

IndexT DiagLinearSubsystem::dof_offset() const noexcept
{
    auto& offset_counts = m_global_linear_system->m_impl.diag_dof_offsets_counts;
    auto offsets = offset_counts.offsets();
    UIPC_ASSERT(m_index < offsets.size(),
                "Invalid subsystem index, Linear Subsystem Count = {}, Your Subsystem ({}) Index = {}",
                offsets.size(),
                name(),
                m_index);
    return offsets[m_index];
}

IndexT DiagLinearSubsystem::dof_count() const noexcept
{
    auto& offset_counts = m_global_linear_system->m_impl.diag_dof_offsets_counts;
    auto counts = offset_counts.counts();
    UIPC_ASSERT(m_index < counts.size(),
                "Invalid subsystem index, Linear Subsystem Count = {}, Your Subsystem ({}) Index = {}",
                counts.size(),
                name(),
                m_index);
    return counts[m_index];
}

void DiagLinearSubsystem::do_build(BuildInfo& info) {}

void DiagLinearSubsystem::do_build()
{
    m_global_linear_system = require<GlobalLinearSystem>();

    BuildInfo info;
    do_build(info);

    m_global_linear_system->add_subsystem(this);
}

void DiagLinearSubsystem::init()
{
    InitInfo info;
    do_init(info);
}

void DiagLinearSubsystem::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    do_report_init_extent(info);
}

void DiagLinearSubsystem::receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    do_receive_init_dof_info(info);
}

void DiagLinearSubsystem::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    do_report_extent(info);
}

void DiagLinearSubsystem::assemble(GlobalLinearSystem::DiagInfo& info)
{
    UIPC_ASSERT(info.gradient_only()
                    || info.component_flags() == GlobalLinearSystem::ComponentFlags::All,
                "Limitation: When info.gradient_only()==false, info.component_flags() must be GlobalLinearSystem::ComponentFlags::All (got {})",
                enum_flags_name(info.component_flags()));

    do_assemble(info);
}

void DiagLinearSubsystem::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    do_accuracy_check(info);
}
void DiagLinearSubsystem::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    do_retrieve_solution(info);
}
}  // namespace uipc::backend::cuda