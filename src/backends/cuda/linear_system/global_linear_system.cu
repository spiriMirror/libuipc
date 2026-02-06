#include <linear_system/global_linear_system.h>
#include <linear_system/diag_linear_subsystem.h>
#include <linear_system/off_diag_linear_subsystem.h>
#include <uipc/common/range.h>
#include <linear_system/iterative_solver.h>
#include <linear_system/global_preconditioner.h>
#include <linear_system/local_preconditioner.h>
#include <fstream>
#include <sim_engine.h>
#include <backends/common/backend_path_tool.h>
#include <Eigen/Sparse>
#include <utils/matrix_market.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(GlobalLinearSystem);

SizeT GlobalLinearSystem::dof_count() const
{
    return m_impl.diag_dof_offsets_counts.total_count();
}

void GlobalLinearSystem::do_build()
{
    auto dump_linear_system_attr =
        world().scene().config().find<IndexT>("extras/debug/dump_linear_system");

    m_impl.need_debug_dump =
        dump_linear_system_attr ? dump_linear_system_attr->view()[0] : false;
}

void GlobalLinearSystem::_dump_A_b()
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    auto output_path_A = fmt::format("{}A.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter());
    export_matrix_market(output_path_A, m_impl.bcoo_A.cview());
    logger::info("Dumped global linear system matrix A to {}", output_path_A);

    auto output_path_b = fmt::format("{}b.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter());
    export_vector_market(output_path_b, m_impl.b.cview());
    logger::info("Dumped global linear system vector b to {}", output_path_b);
}

void GlobalLinearSystem::_dump_x()
{
    auto path_tool = BackendPathTool(workspace());
    auto output_folder = path_tool.workspace(UIPC_RELATIVE_SOURCE_FILE, "debug");
    export_vector_market(fmt::format("{}x.{}.{}.mtx",
                                     output_folder.string(),
                                     engine().frame(),
                                     engine().newton_iter()),
                         m_impl.x.cview());
}


void GlobalLinearSystem::solve()
{
    m_impl.build_linear_system();

    if(m_impl.empty_system) [[unlikely]]
        return;

    logger::info("GlobalLinearSystem has {} DoFs, Unique Triplet Count: {}",
                 m_impl.b.size(),
                 m_impl.bcoo_A.triplet_count());

    if(m_impl.need_debug_dump) [[unlikely]]
        _dump_A_b();

    m_impl.solve_linear_system();

    if(m_impl.need_debug_dump) [[unlikely]]
        _dump_x();

    m_impl.distribute_solution();
}

void GlobalLinearSystem::Impl::init()
{
    // 1) Init all diag subsystems and off-diag subsystems

    auto diag_subsystem_view     = diag_subsystems.view();
    auto off_diag_subsystem_view = off_diag_subsystems.view();

    {
        // Sort the diag subsystems by their UIDs to ensure the order is consistent
        // ref: https://github.com/spiriMirror/libuipc/issues/271
        std::ranges::sort(diag_subsystem_view,
                          [](const DiagLinearSubsystem* a, const DiagLinearSubsystem* b)
                          { return a->uid() < b->uid(); });
        std::ranges::sort(off_diag_subsystem_view,
                          [](const OffDiagLinearSubsystem* a,
                             const OffDiagLinearSubsystem* b) -> bool
                          { return a->uid() < b->uid(); });
    }


    auto total_count = diag_subsystem_view.size() + off_diag_subsystem_view.size();
    subsystem_infos.resize(total_count);

    // Diag System Always Go First
    auto diag_span = span{subsystem_infos}.subspan(0, diag_subsystem_view.size());
    // Off Diag System Always After Diag System
    auto off_diag_span = span{subsystem_infos}.subspan(diag_subsystem_view.size(),
                                                       off_diag_subsystem_view.size());
    {
        auto offset = 0;
        for(auto i : range(diag_span.size()))
        {
            auto& dst_diag                  = diag_span[i];
            dst_diag.is_diag                = true;
            dst_diag.local_index            = i;
            auto index                      = offset + i;
            dst_diag.index                  = index;
            diag_subsystem_view[i]->m_index = index;
        }

        offset += diag_subsystem_view.size();
        for(auto i : range(off_diag_span.size()))
        {
            auto& dst_off_diag       = off_diag_span[i];
            dst_off_diag.is_diag     = false;
            dst_off_diag.local_index = i;
            dst_off_diag.index       = offset + i;
        }

        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
            diag_subsystem->init();

        for(auto&& [i, off_diag_subsystem] : enumerate(off_diag_subsystem_view))
            off_diag_subsystem->init();
    }


    // 2) DoF Offsets/Counts
    {
        diag_dof_offsets_counts.resize(diag_subsystem_view.size());
        auto diag_dof_counts = diag_dof_offsets_counts.counts();
        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
        {
            InitDofExtentInfo info;
            diag_subsystem->report_init_extent(info);
            diag_dof_counts[i] = info.m_dof_count;
        }
        diag_dof_offsets_counts.scan();
        auto diag_dof_offsets = diag_dof_offsets_counts.offsets();
        for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
        {
            InitDofInfo info;
            info.m_dof_offset = diag_dof_offsets[i];
            info.m_dof_count  = diag_dof_counts[i];
            diag_subsystem->receive_init_dof_info(info);
        }
    }
    accuracy_statisfied_flags.resize(diag_subsystem_view.size());

    // 3) Triplet Offsets/Counts
    subsystem_triplet_offsets_counts.resize(total_count);
    off_diag_lr_triplet_counts.resize(off_diag_subsystem_view.size());

    // 4) Preconditioner
    // find out diag systems that don't have preconditioner
    auto local_preconditioner_view = local_preconditioners.view();

    for(auto precond : local_preconditioner_view)
    {
        auto index = precond->m_subsystem->m_index;
        diag_span[index].has_local_preconditioner = true;
    }

    no_precond_diag_subsystem_indices.reserve(diag_span.size());
    for(auto&& [i, diag_info] : enumerate(diag_span))
    {
        if(!diag_info.has_local_preconditioner)
        {
            no_precond_diag_subsystem_indices.push_back(i);
        }
    }

    for(auto precond : local_preconditioner_view)
    {
        precond->init();
    }
}

void GlobalLinearSystem::Impl::build_linear_system()
{
    Timer timer{"Build Linear System"};
    empty_system = !_update_subsystem_extent();

    if(empty_system) [[unlikely]]
    {
        logger::warn("The global linear system is empty, skip *assembling, *solving and *solution distributing phase.");
        return;
    }

    _assemble_linear_system();

    converter.ge2sym(triplet_A);
    converter.convert(triplet_A, bcoo_A);

    _assemble_preconditioner();

    logger::info("GlobalLinearSystem has {} DoFs, Unique Triplet Count: {}",
                 b.size(),
                 bcoo_A.triplet_count());
}

bool GlobalLinearSystem::Impl::_update_subsystem_extent()
{
    bool dof_count_changed     = false;
    bool triplet_count_changed = false;

    auto diag_subsystem_view       = diag_subsystems.view();
    auto off_diag_subsystem_view   = off_diag_subsystems.view();
    auto diag_dof_counts           = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets          = diag_dof_offsets_counts.offsets();
    auto subsystem_triplet_counts  = subsystem_triplet_offsets_counts.counts();
    auto subsystem_triplet_offsets = subsystem_triplet_offsets_counts.offsets();

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto           dof_i          = subsystem_info.local_index;
            auto           triplet_i      = subsystem_info.index;
            auto&          diag_subsystem = diag_subsystem_view[dof_i];
            DiagExtentInfo info;
            diag_subsystem->report_extent(info);

            dof_count_changed |= diag_dof_counts[dof_i] != info.m_dof_count;
            diag_dof_counts[dof_i] = info.m_dof_count;


            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != info.m_block_count;
            subsystem_triplet_counts[triplet_i] = info.m_block_count;
        }
        else
        {
            auto triplet_i = subsystem_info.index;
            auto& off_diag_subsystem = off_diag_subsystem_view[subsystem_info.local_index];
            OffDiagExtentInfo info;
            off_diag_subsystem->report_extent(info);

            auto total_block_count = info.m_lr_block_count + info.m_rl_block_count;

            triplet_count_changed |= subsystem_triplet_counts[triplet_i] != total_block_count;
            subsystem_triplet_counts[triplet_i] = total_block_count;
            off_diag_lr_triplet_counts[subsystem_info.local_index] =
                SizeT2{info.m_lr_block_count, info.m_rl_block_count};
        }
    }

    SizeT total_dof     = 0;
    SizeT total_triplet = 0;

    if(dof_count_changed)
    {
        diag_dof_offsets_counts.scan();
    }
    total_dof = diag_dof_offsets_counts.total_count();
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

    if(triplet_count_changed) [[likely]]
    {
        subsystem_triplet_offsets_counts.scan();
    }
    total_triplet = subsystem_triplet_offsets_counts.total_count();

    if(triplet_A.triplet_capacity() < total_triplet)
    {
        auto reserve_count = total_triplet * reserve_ratio;
        triplet_A.reserve_triplets(reserve_count);
        bcoo_A.reserve_triplets(reserve_count);
    }
    triplet_A.resize_triplets(total_triplet);

    if(total_dof == 0 || total_triplet == 0) [[unlikely]]
    {
        return false;
    }

    return true;
}

void GlobalLinearSystem::Impl::_assemble_linear_system()
{
    auto HA = triplet_A.view();

    // Clear and invalidate previous values
    triplet_A.values().fill(Matrix3x3::Zero());
    triplet_A.row_indices().fill(-1);
    triplet_A.col_indices().fill(-1);

    auto B = b.view();
    B.buffer_view().fill(0.0);

    auto diag_subsystem_view     = diag_subsystems.view();
    auto off_diag_subsystem_view = off_diag_subsystems.view();

    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    auto subsystem_triplet_counts  = subsystem_triplet_offsets_counts.counts();
    auto subsystem_triplet_offsets = subsystem_triplet_offsets_counts.offsets();

    for(const auto& subsystem_info : subsystem_infos)
    {
        if(subsystem_info.is_diag)
        {
            auto  dof_i          = subsystem_info.local_index;
            auto  triplet_i      = subsystem_info.index;
            auto& diag_subsystem = diag_subsystem_view[dof_i];

            int  dof_offset         = diag_dof_offsets[dof_i];
            int  dof_count          = diag_dof_counts[dof_i];
            int  blocked_dof_offset = dof_offset / DoFBlockSize;
            int  blocked_dof_count  = dof_count / DoFBlockSize;
            int2 ij_offset          = {blocked_dof_offset, blocked_dof_offset};
            int2 ij_count           = {blocked_dof_count, blocked_dof_count};

            DiagInfo info{this};

            info.m_index     = triplet_i;
            info.m_gradients = B.subview(dof_offset, dof_count);
            info.m_hessians  = HA.subview(subsystem_triplet_offsets[triplet_i],
                                         subsystem_triplet_counts[triplet_i])
                                  .submatrix(ij_offset, ij_count);

            diag_subsystem->assemble(info);
        }
        else
        {
            auto triplet_i   = subsystem_info.index;
            auto local_index = subsystem_info.local_index;
            auto& off_diag_subsystem = off_diag_subsystem_view[subsystem_info.local_index];
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
            info.m_index = triplet_i;

            info.m_lr_hessian =
                HA.subview(lr_triplet_offset, lr_triplet_count)
                    .submatrix(int2{l_blocked_dof_offset, r_blocked_dof_offset},
                               int2{l_blocked_dof_count, r_blocked_dof_count});

            info.m_rl_hessian =
                HA.subview(rl_triplet_offset, rl_triplet_count)
                    .submatrix(int2{r_blocked_dof_offset, l_blocked_dof_offset},
                               int2{r_blocked_dof_count, l_blocked_dof_count});

            // logger::info("rl_offset: {}, lr_offset: {}", rl_triplet_offset, lr_triplet_offset);

            off_diag_subsystem->assemble(info);
        }
    }
}

void GlobalLinearSystem::Impl::_assemble_preconditioner()
{
    if(global_preconditioner)
    {
        GlobalPreconditionerAssemblyInfo info{this};
        global_preconditioner->assemble(info);
    }

    for(auto&& preconditioner : local_preconditioners.view())
    {
        LocalPreconditionerAssemblyInfo info{this, preconditioner->m_subsystem->m_index};
        preconditioner->assemble(info);
    }
}

void GlobalLinearSystem::Impl::solve_linear_system()
{
    Timer timer{"Solve Linear System"};
    if(iterative_solver)
    {
        SolvingInfo info{this};
        info.m_b = b.cview();
        info.m_x = x.view();
        iterative_solver->solve(info);
        logger::info("Iterative linear solver iteration count: {}", info.m_iter_count);
    }
}

void GlobalLinearSystem::Impl::distribute_solution()
{
    auto diag_subsystem_view = diag_subsystems.view();
    auto diag_dof_counts     = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets    = diag_dof_offsets_counts.offsets();

    // distribute the solution to all diag subsystems
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystems.view()))
    {
        SolutionInfo info{this};
        info.m_solution = x.view().subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystem->retrieve_solution(info);
    }
}

void GlobalLinearSystem::Impl::apply_preconditioner(muda::DenseVectorView<Float> z,
                                                    muda::CDenseVectorView<Float> r)
{
    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    if(global_preconditioner)
    {
        ApplyPreconditionerInfo info{this};
        info.m_z = z;
        info.m_r = r;
        global_preconditioner->apply(info);
    }

    for(auto& preconditioner : local_preconditioners.view())
    {
        ApplyPreconditionerInfo info{this};
        auto                    index  = preconditioner->m_subsystem->m_index;
        auto                    offset = diag_dof_offsets[index];
        auto                    count  = diag_dof_counts[index];
        info.m_z                       = z.subview(offset, count);
        info.m_r                       = r.subview(offset, count);
        preconditioner->apply(info);
    }

    if(!global_preconditioner)
    {
        // For diag subsystems without local preconditioner, just copy r to z
        for(auto i : no_precond_diag_subsystem_indices)
        {
            auto offset = diag_dof_offsets[i];
            auto count  = diag_dof_counts[i];
            auto z_sub  = z.subview(offset, count);
            auto r_sub  = r.subview(offset, count);
            z_sub.buffer_view().copy_from(r_sub.buffer_view());
        }
    }
}

void GlobalLinearSystem::Impl::spmv(Float                         a,
                                    muda::CDenseVectorView<Float> x,
                                    Float                         b,
                                    muda::DenseVectorView<Float>  y)
{
    spmver.rbk_sym_spmv(a, bcoo_A.cview(), x, b, y);

    // Just some debug options
    //  * spmver.sym_spmv(a, bcoo_A.cview(), x, b, y);      // Slightly slower
    //  * spmver.cpu_sym_spmv(a, bcoo_A.cview(), x, b, y);  // Much slower
}

bool GlobalLinearSystem::Impl::accuracy_statisfied(muda::DenseVectorView<Float> r)
{
    auto diag_dof_counts  = diag_dof_offsets_counts.counts();
    auto diag_dof_offsets = diag_dof_offsets_counts.offsets();

    for(auto&& [i, diag_subsystems] : enumerate(diag_subsystems.view()))
    {
        AccuracyInfo info{this};
        info.m_r = r.subview(diag_dof_offsets[i], diag_dof_counts[i]);
        diag_subsystems->accuracy_check(info);

        accuracy_statisfied_flags[i] = info.m_statisfied ? 1 : 0;
    }

    return std::ranges::all_of(accuracy_statisfied_flags,
                               [](bool flag) { return flag; });
}

void GlobalLinearSystem::Impl::compute_gradient(ComputeGradientInfo& info)
{
    auto diag_subsystem_view = diag_subsystems.view();

    // report extent first
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        DiagExtentInfo diag_info;
        diag_info.m_gradient_only   = true;
        diag_info.m_component_flags = info.m_flags;
        diag_subsystem->report_extent(diag_info);
    }

    // assemble gradient
    for(auto&& [i, diag_subsystem] : enumerate(diag_subsystem_view))
    {
        DiagInfo diag_info{this};
        diag_info.m_index         = diag_subsystem->m_index;
        diag_info.m_gradients     = info.m_gradients;
        diag_info.m_hessians      = TripletMatrixView{};  // Empty Hessian View
        diag_info.m_gradient_only = true;
        diag_info.m_component_flags = info.m_flags;
        diag_subsystem->assemble(diag_info);
    }
}

void GlobalLinearSystem::DiagExtentInfo::extent(SizeT hessian_count, SizeT dof_count) noexcept
{

    UIPC_ASSERT(!(m_gradient_only && hessian_count != 0),
                "When gradient_only is true, hessian_count must be 0, yours {}.",
                hessian_count);


    m_block_count = hessian_count;
    UIPC_ASSERT(dof_count % DoFBlockSize == 0,
                "dof_count must be multiple of {}, yours {}.",
                DoFBlockSize,
                dof_count);
    m_dof_count = dof_count;
}

void GlobalLinearSystem::OffDiagExtentInfo::extent(SizeT lr_hessian_block_count,
                                                   SizeT rl_hassian_block_count) noexcept
{
    m_lr_block_count = lr_hessian_block_count;
    m_rl_block_count = rl_hassian_block_count;
}
auto GlobalLinearSystem::AssemblyInfo::A() const -> CBCOOMatrixView
{
    return m_impl->bcoo_A.cview();
}

SizeT GlobalLinearSystem::LocalPreconditionerAssemblyInfo::dof_offset() const
{
    auto diag_dof_offsets = m_impl->diag_dof_offsets_counts.offsets();
    return diag_dof_offsets[m_index];
}

SizeT GlobalLinearSystem::LocalPreconditionerAssemblyInfo::dof_count() const
{
    auto diag_dof_counts = m_impl->diag_dof_offsets_counts.counts();
    return diag_dof_counts[m_index];
}

void GlobalLinearSystem::compute_gradient(ComputeGradientInfo& info)
{
    m_impl.compute_gradient(info);
}
}  // namespace uipc::backend::cuda

namespace uipc::backend::cuda
{
void GlobalLinearSystem::ComputeGradientInfo::buffer_view(muda::DenseVectorView<Float> grad) noexcept
{
    m_gradients = grad;
}

void GlobalLinearSystem::ComputeGradientInfo::flags(ComponentFlags flags) noexcept
{
    m_flags = flags;
}

void GlobalLinearSystem::add_subsystem(DiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    UIPC_ASSERT(subsystem != nullptr, "The subsystem should not be nullptr.");
    m_impl.diag_subsystems.register_sim_system(*subsystem);
}

void GlobalLinearSystem::add_subsystem(OffDiagLinearSubsystem* subsystem)
{
    check_state(SimEngineState::BuildSystems, "add_subsystem()");
    m_impl.off_diag_subsystems.register_sim_system(*subsystem);
}

void GlobalLinearSystem::add_solver(IterativeSolver* solver)
{
    check_state(SimEngineState::BuildSystems, "add_solver()");
    UIPC_ASSERT(solver != nullptr, "The solver should not be nullptr.");
    m_impl.iterative_solver.register_sim_system(*solver);
}

void GlobalLinearSystem::add_preconditioner(LocalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(preconditioner != nullptr, "The preconditioner should not be nullptr.");
    m_impl.local_preconditioners.register_sim_system(*preconditioner);
}

void GlobalLinearSystem::add_preconditioner(GlobalPreconditioner* preconditioner)
{
    check_state(SimEngineState::BuildSystems, "add_preconditioner()");
    UIPC_ASSERT(preconditioner != nullptr, "The preconditioner should not be nullptr.");
    m_impl.global_preconditioner.register_sim_system(*preconditioner);
}

void GlobalLinearSystem::init()
{
    m_impl.init();
}
}  // namespace uipc::backend::cuda