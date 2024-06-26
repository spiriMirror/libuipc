#include <affine_body/abd_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen.h>

namespace uipc::backend::cuda
{
template <>
class SimSystemCreator<ABDLinearSubsystem>
{
  public:
    static U<ABDLinearSubsystem> create(SimEngine& engine)
    {
        return has_affine_body_constitution(engine) ?
                   make_unique<ABDLinearSubsystem>(engine) :
                   nullptr;
    }
};

REGISTER_SIM_SYSTEM(ABDLinearSubsystem);

void ABDLinearSubsystem::do_build()
{
    m_impl.m_abd = find<AffineBodyDynamics>();
    UIPC_ASSERT(m_impl.m_abd, "ABDLinearSubsystem requires AffineBodyDynamics");
    auto global_linear_system = find<GlobalLinearSystem>();
    global_linear_system->add_subsystem(this);
}


void ABDLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Full,
                "Now only support Full Hessian");

    constexpr SizeT M12x12_to_M3x3 = (12 * 12) / (3 * 3);
    hessian_block_count = abd().body_id_to_body_hessian.size() * M12x12_to_M3x3;
    dof_count           = abd().abd_body_count * 12;

    info.extent(hessian_block_count, dof_count);
}

void ABDLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    _assemble_gradient(info);
    _assemble_hessian(info);
}

void ABDLinearSubsystem::Impl::_assemble_gradient(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(abd().body_count(),
               [abd_gradient = abd().body_id_to_body_gradient.cviewer().name("abd_gradient"),
                gradient = info.gradient().viewer().name("gradient")] __device__(int i) mutable
               { gradient.segment<12>(i * 12) = abd_gradient(i); });
}

void ABDLinearSubsystem::Impl::_assemble_hessian(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;

    _make_hessian_unique();

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(bcoo_A.non_zero_blocks(),
               [dst = info.hessian().viewer().name("hessian"),
                src = bcoo_A.cviewer().name("bcoo_hessian")] __device__(int I) mutable
               {
                   auto offset = I * 16;
                   for(int i = 0; i < 4; ++i)
                       for(int j = 0; j < 4; ++j)
                       {
                           auto&& [row, col, H12x12] = src(I);
                           dst(offset++).write(row * 4 + i,
                                               col * 4 + j,
                                               H12x12.block<3, 3>(3 * i, 3 * j));
                       }
               });
}

void ABDLinearSubsystem::Impl::_make_hessian_unique()
{
    using namespace muda;

    if(triplet_A.triplet_count() < hessian_block_count)
    {
        auto reserve_count = hessian_block_count * reserve_ratio;
        triplet_A.reserve_triplets(reserve_count);
        triplet_A.resize_triplets(hessian_block_count);
        bcoo_A.reserve_triplets(reserve_count);
        bcoo_A.resize_triplets(hessian_block_count);
    }

    SizeT blocked_dofs = dof_count / 12;
    triplet_A.reshape(blocked_dofs, blocked_dofs);

    auto A = triplet_A.view();

    auto offset = 0;
    {
        auto count = abd().body_id_to_body_hessian.size();
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(count,
                   [body_hessian = abd().body_id_to_body_hessian.cviewer().name("body_hessian"),
                    triplet = A.subview(offset, count).viewer().name("triplet")] __device__(int i) mutable
                   { triplet(i).write(i, i, body_hessian(i)); });

        offset += count;
    }

    converter.convert(triplet_A, bcoo_A);
}

void ABDLinearSubsystem::Impl::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    info.statisfied(true);
}

void ABDLinearSubsystem::Impl::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    using namespace muda;

    auto dq = abd().body_id_to_dq.view();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(abd().body_count(),
               [dq = dq.viewer().name("dq"),
                x = info.solution().viewer().name("x")] __device__(int i) mutable
               {
                   dq(i) = -x.segment<12>(i * 12).as_eigen();
                   // cout << "solution dq: \n" << dq(i) << "\n";
               });
}
}  // namespace uipc::backend::cuda


namespace uipc::backend::cuda
{
void ABDLinearSubsystem::do_report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    m_impl.report_extent(info);
}

void ABDLinearSubsystem::do_assemble(GlobalLinearSystem::DiagInfo& info)
{
    m_impl.assemble(info);
}

void ABDLinearSubsystem::do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    m_impl.accuracy_check(info);
}

void ABDLinearSubsystem::do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    m_impl.retrieve_solution(info);
}

}  // namespace uipc::backend::cuda
