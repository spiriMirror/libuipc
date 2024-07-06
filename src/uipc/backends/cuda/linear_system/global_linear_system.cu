#include <linear_system/global_linear_system.h>
#include <linear_system/diag_linear_subsystem.h>
#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/range.h>
#include <linear_system/iterative_solver.h>
#include <linear_system/global_preconditioner.h>
#include <linear_system/local_preconditioner.h>

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
    // if the system is empty, skip the following steps
    if(m_impl.empty_system) [[unlikely]]
        return;
    m_impl.solve_linear_system();
    m_impl.distribute_solution();
}

void GlobalLinearSystem::Impl::init()
{
    UIPC_ASSERT(!diag_subsystem_buffer.empty(),
                "No linear subsystems added to the global linear system.");

    // build the linear subsystem infos
    diag_subsystems.resize(diag_subsystem_buffer.size());
    off_diag_subsystems.resize(off_diag_subsystem_buffer.size());
    local_preconditioners.resize(local_preconditioner_buffer.size());

    std::ranges::move(diag_subsystem_buffer, diag_subsystems.begin());
    std::ranges::move(off_diag_subsystem_buffer, off_diag_subsystems.begin());
    std::ranges::move(local_preconditioner_buffer, local_preconditioners.begin());

    auto total_count = diag_subsystems.size() + off_diag_subsystems.size();

    subsystem_infos.resize(total_count);

    // put the diag subsystems in the front
    auto diag_span = span{subsystem_infos}.subspan(0, diag_subsystems.size());
    // then the off diag subsystems
    auto off_diag_span = span{subsystem_infos}.subspan(diag_subsystems.size(),
                                                       off_diag_subsystems.size());

    auto offset = 0;
    for(auto i : range(diag_span.size()))
    {
        auto& dst_diag              = diag_span[i];
        dst_diag.is_diag            = true;
        dst_diag.local_index        = i;
        auto index                  = offset + i;
        dst_diag.index              = index;
        diag_subsystems[i]->m_index = index;
    }

    offset += diag_subsystems.size();
    for(auto i : range(off_diag_span.size()))
    {
        auto& dst_off_diag       = off_diag_span[i];
        dst_off_diag.is_diag     = false;
        dst_off_diag.local_index = i;
        dst_off_diag.index       = offset + i;
    }

    // prepare the storage for dof and matrix triplet
    subsystem_triplet_offsets.resize(total_count, ~0ull);
    subsystem_triplet_counts.resize(total_count, ~0ull);


    diag_dof_offsets.resize(diag_subsystems.size());
    diag_dof_counts.resize(diag_subsystems.size());
    accuracy_statisfied_flags.resize(diag_subsystems.size());

    off_diag_lr_triplet_counts.resize(off_diag_subsystems.size());
}

void GlobalLinearSystem::Impl::build_linear_system()
{
    empty_system = !_update_subsystem_extent();
    // if empty, skip the following steps
    if(empty_system) [[unlikely]]
        return;

    _assemble_linear_system();

    converter.convert(triplet_A, bcoo_A);
    converter.ge2sym(bcoo_A);

    _assemble_preconditioner();
}

bool GlobalLinearSystem::Impl::_update_subsystem_extent()
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
            info.m_storage_type = HessianStorageType::Full;
            diag_subsystem->report_extent(info);

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
            info.m_storage_type = HessianStorageType::Full;
            off_diag_subsystem->report_extent(info);

            auto total_block_count = info.m_lr_block_count + info.m_rl_block_count;

            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != total_block_count;
            subsystem_triplet_counts[triplet_i] = total_block_count;
            off_diag_lr_triplet_counts[subsystem_info.local_index] =
                ulonglong2{info.m_lr_block_count, info.m_rl_block_count};
        }
    }

    SizeT total_dof     = 0;
    SizeT total_triplet = 0;

    if(dof_count_changed)
    {
        std::exclusive_scan(
            diag_dof_counts.begin(), diag_dof_counts.end(), diag_dof_offsets.begin(), 0);
        total_dof = diag_dof_offsets.back() + diag_dof_counts.back();
        if(x.capacity() < total_dof)
        {
            auto reserve_count = total_dof * reserve_ratio;
            x.reserve(reserve_count);
            b.reserve(reserve_count);
        }
        auto blocked_dof = total_dof / DoFBlockSize;
        triplet_A.reshape(blocked_dof, blocked_dof);
        x.resize(total_dof);
        b.resize(total_dof);
    }
    else
    {
        total_dof = diag_dof_offsets.back() + diag_dof_counts.back();
    }

    if(triplet_count_changed) [[likely]]
    {
        std::exclusive_scan(subsystem_triplet_counts.begin(),
                            subsystem_triplet_counts.end(),
                            subsystem_triplet_offsets.begin(),
                            0);
        total_triplet =
            subsystem_triplet_offsets.back() + subsystem_triplet_counts.back();
        if(triplet_A.triplet_capacity() < total_triplet)
        {
            auto reserve_count = total_triplet * reserve_ratio;
            triplet_A.reserve_triplets(reserve_count);
            bcoo_A.reserve_triplets(reserve_count);
        }
        triplet_A.resize_triplets(total_triplet);
    }
    else
    {
        total_triplet =
            subsystem_triplet_offsets.back() + subsystem_triplet_counts.back();
    }

    if(total_dof == 0 || total_triplet == 0) [[unlikely]]
    {
        spdlog::warn("The global linear system is empty, skip *assembling, *solving and *solution distributing phase.");
        return false;
    }

    return true;
}

void GlobalLinearSystem::Impl::_assemble_linear_system()
{
    auto HA = triplet_A.view();
    auto B  = b.view();
    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto  dof_i          = subsystem_info.local_index;
            auto  triplet_i      = subsystem_info.index;
            auto& diag_subsystem = diag_subsystems[dof_i];

            int  dof_offset         = diag_dof_offsets[dof_i];
            int  dof_count          = diag_dof_counts[dof_i];
            int  blocked_dof_offset = dof_offset / DoFBlockSize;
            int  blocked_dof_count  = dof_count / DoFBlockSize;
            int2 ij_offset          = {blocked_dof_offset, blocked_dof_offset};
            int2 ij_count           = {blocked_dof_count, blocked_dof_count};

            DiagInfo info{this};

            info.m_index        = triplet_i;
            info.m_storage_type = HessianStorageType::Full;
            info.m_gradient     = B.subview(dof_offset, dof_count);
            info.m_hessian = HA.subview(subsystem_triplet_offsets[triplet_i],
                                        subsystem_triplet_counts[triplet_i])
                                 .submatrix(ij_offset, ij_count);

            diag_subsystem->assemble(info);
        }
        else
        {
            auto triplet_i   = subsystem_info.index;
            auto local_index = subsystem_info.local_index;
            auto& off_diag_subsystem = off_diag_subsystems[subsystem_info.local_index];
            auto& l_diag_index = off_diag_subsystem->m_l->m_index;
            auto& r_diag_index = off_diag_subsystem->m_r->m_index;


            int l_blocked_dof_offset = diag_dof_offsets[l_diag_index] / DoFBlockSize;
            int l_blocked_dof_count = diag_dof_counts[l_diag_index] / DoFBlockSize;

            int r_blocked_dof_offset = diag_dof_offsets[r_diag_index] / DoFBlockSize;
            int r_blocked_dof_count = diag_dof_counts[r_diag_index] / DoFBlockSize;

            auto lr_triplet_offset = subsystem_triplet_offsets[triplet_i];
            auto lr_triplet_count  = off_diag_lr_triplet_counts[local_index].x;
            auto rl_triplet_offset = lr_triplet_offset + lr_triplet_count;
            auto rl_triplet_count  = off_diag_lr_triplet_counts[local_index].y;

            OffDiagInfo info{this};
            info.m_index        = triplet_i;
            info.m_storage_type = HessianStorageType::Full;

            info.m_lr_hessian =
                HA.subview(lr_triplet_offset, lr_triplet_count)
                    .submatrix(int2{l_blocked_dof_offset, r_blocked_dof_offset},
                               int2{l_blocked_dof_count, r_blocked_dof_count});

            info.m_rl_hessian =
                HA.subview(rl_triplet_offset, rl_triplet_count)
                    .submatrix(int2{r_blocked_dof_offset, l_blocked_dof_offset},
                               int2{r_blocked_dof_count, l_blocked_dof_count});

            off_diag_subsystem->assemble(info);
        }
    }
}

void GlobalLinearSystem::Impl::_assemble_preconditioner()
{
    if(global_preconditioner)
    {
        GlobalPreconditionerAssemblyInfo info{this};
        info.symmetric = true;
        global_preconditioner->assemble(info);
    }

    for(auto&& [i, preconditioner] : enumerate(local_preconditioners))
    {
        LocalPreconditionerAssemblyInfo info{this};
        info.m_index = i;
        preconditioner->assemble(info);
    }
}

void GlobalLinearSystem::Impl::solve_linear_system()
{
    if(iterative_solver)
    {
        SolvingInfo info{this};
        info.m_b = b.cview();
        info.m_x = x.view();
        iterative_solver->solve(info);
    }
}

void GlobalLinearSystem::Impl::distribute_solution()
{
    // _distribute the solution to all diag subsystems
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems))
    {
        SolutionInfo info{this};
        info.m_solution = x.view().subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystem->retrieve_solution(info);
    }
}

void GlobalLinearSystem::Impl::apply_preconditioner(muda::DenseVectorView<Float> z,
                                                    muda::CDenseVectorView<Float> r)
{
    if(global_preconditioner)
    {
        ApplyPreconditionerInfo info{this};
        info.m_z = z;
        info.m_r = r;
        global_preconditioner->apply(info);
    }

    for(auto& preconditioner : local_preconditioners)
    {
        ApplyPreconditionerInfo info{this};
        auto                    index  = preconditioner->m_subsystem->m_index;
        auto                    offset = diag_dof_offsets[index];
        auto                    count  = diag_dof_counts[index];
        info.m_z                       = z.subview(offset, count);
        info.m_r                       = r.subview(offset, count);
        preconditioner->apply(info);
    }

    if(!global_preconditioner && local_preconditioners.empty())
    {
        muda::BufferLaunch().copy<Float>(z.buffer_view(), r.buffer_view());
    }
}

void GlobalLinearSystem::Impl::spmv(Float                         a,
                                    muda::CDenseVectorView<Float> x,
                                    Float                         b,
                                    muda::DenseVectorView<Float>  y)
{
    spmver.rbk_sym_spmv(a, bcoo_A.cview(), x, b, y);
}

bool GlobalLinearSystem::Impl::accuracy_statisfied(muda::DenseVectorView<Float> r)
{
    for(auto&& [i, diag_subsystems] : enumerate(diag_subsystems))
    {
        AccuracyInfo info{this};
        info.m_r = r.subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystems->accuracy_check(info);

        accuracy_statisfied_flags[i] = info.m_statisfied ? 1 : 0;
    }

    return std::ranges::all_of(accuracy_statisfied_flags,
                               [](bool flag) { return flag; });
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void GlobalLinearSystem::add_subsystem(DiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    m_impl.diag_subsystem_buffer.push_back(subsystem);
}

void GlobalLinearSystem::add_subsystem(OffDiagLinearSubsystem* subsystem,
                                       DiagLinearSubsystem*    depend_l,
                                       DiagLinearSubsystem*    depend_r)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    UIPC_ASSERT(depend_l != nullptr && depend_r != nullptr,
                "The depend_l and depend_r should not be nullptr.");
    subsystem->depend_on(depend_l, depend_r);
    m_impl.off_diag_subsystem_buffer.push_back(subsystem);
}

void GlobalLinearSystem::add_solver(IterativeSolver* solver)
{
    check_state(SimEngineState::BuildSystems, "add_solver()");
    UIPC_ASSERT(m_impl.iterative_solver == nullptr,
                "Only support one linear system solver, {} already added before.",
                m_impl.iterative_solver->name());
    m_impl.iterative_solver = solver;
    solver->m_system        = this;
}

void GlobalLinearSystem::add_preconditioner(LocalPreconditioner* preconditioner,
                                            DiagLinearSubsystem* depend_subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(depend_subsystem != nullptr, "The depend_subsystem should not be nullptr.");
    preconditioner->m_subsystem = depend_subsystem;
    m_impl.local_preconditioner_buffer.push_back(preconditioner);
}

void GlobalLinearSystem::add_preconditioner(GlobalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(m_impl.global_preconditioner == nullptr,
                "Only support one global preconditioner, {} already added before.",
                m_impl.global_preconditioner->name());
    m_impl.global_preconditioner = preconditioner;
}
}  // namespace uipc::backend::cuda