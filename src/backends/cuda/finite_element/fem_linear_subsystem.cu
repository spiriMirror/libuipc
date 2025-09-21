#include <finite_element/fem_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen.h>
#include <muda/ext/eigen/evd.h>
#include <muda/ext/eigen/atomic.h>
#include <finite_element/finite_element_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>
#include <sim_engine.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMLinearSubsystem);

void FEMLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo&)
{
    m_impl.finite_element_method = require<FiniteElementMethod>();
    m_impl.finite_element_vertex_reporter = require<FiniteElementVertexReporter>();
    m_impl.sim_engine = &engine();
    auto dt_attr      = world().scene().config().find<Float>("dt");
    m_impl.dt         = dt_attr->view()[0];

    m_impl.dytopo_effect_receiver  = find<FEMDyTopoEffectReceiver>();
    m_impl.finite_element_animator = find<FiniteElementAnimator>();
    m_impl.converter.reserve_ratio(1.1);
}

void FEMLinearSubsystem::do_init(DiagLinearSubsystem::InitInfo& info) {}

void FEMLinearSubsystem::Impl::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    info.extent(fem().xs.size() * 3);
}

void FEMLinearSubsystem::Impl::receive_init_dof_info(WorldVisitor& w,
                                                     GlobalLinearSystem::InitDofInfo& info)
{
    auto& geo_infos = fem().geo_infos;
    auto  geo_slots = w.scene().geometries();

    IndexT offset = info.dof_offset();

    finite_element_method->for_each(
        geo_slots,
        [&](const FiniteElementMethod::ForEachInfo& foreach_info, geometry::SimplicialComplex& sc)
        {
            auto I          = foreach_info.global_index();
            auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
            UIPC_ASSERT(dof_offset, "dof_offset not found on FEM mesh why can it happen?");
            auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
            UIPC_ASSERT(dof_count, "dof_count not found on FEM mesh why can it happen?");

            IndexT this_dof_count = 3 * sc.vertices().size();
            view(*dof_offset)[0]  = offset;
            view(*dof_count)[0]   = this_dof_count;

            offset += this_dof_count;
        });

    UIPC_ASSERT(offset == info.dof_offset() + info.dof_count(), "dof size mismatch");
}

void FEMLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Full,
                "Now only support Full Hessian");

    // 1) Hessian Count
    energy_producer_hessian_offset = 0;
    energy_producer_hessian_count  = fem().energy_producer_total_hessian_count;
    auto hessian_block_count       = energy_producer_hessian_count;

    if(dytopo_effect_receiver)  // if dytopo_effect enabled
    {
        dytopo_effect_hessian_offset = hessian_block_count;
        dytopo_effect_hessian_count = dytopo_effect_receiver->hessians().triplet_count();
        hessian_block_count += dytopo_effect_hessian_count;
    }

    if(finite_element_animator)
    {
        FiniteElementAnimator::ExtentInfo extent_info;
        finite_element_animator->report_extent(extent_info);
        animator_hessian_offset = hessian_block_count;
        animator_hessian_count  = extent_info.hessian_block_count;
        hessian_block_count += animator_hessian_count;
    }

    // 2) Gradient Count
    auto dof_count = fem().dxs.size() * 3;

    info.extent(hessian_block_count, dof_count);
}

void FEMLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    // 0) record dof info
    auto frame = sim_engine->frame();
    fem().set_dof_info(frame, info.gradients().offset(), info.gradients().size());

    // 1) Clear Gradient
    info.gradients().buffer_view().fill(0);

    // 2) Assemble Gradient and Hessian
    _assemble_producers(info);
    _assemble_dytopo_effect(info);
    _assemble_animation(info);

    using namespace muda;

    // 3) Clear Fixed Vertex Gradient
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(fem().xs.size(),
               [is_fixed = fem().is_fixed.cviewer().name("is_fixed"),
                gradients = info.gradients().viewer().name("gradients")] __device__(int i) mutable
               {
                   if(is_fixed(i))
                   {
                       gradients.segment<3>(i * 3).as_eigen().setZero();
                   }
               });

    // 4) Clear Fixed Vertex hessian
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(info.hessians().triplet_count(),
               [is_fixed = fem().is_fixed.cviewer().name("is_fixed"),
                hessians = info.hessians().viewer().name("hessians")] __device__(int I) mutable
               {
                   auto&& [i, j, H3] = hessians(I).read();

                   if(is_fixed(i) || is_fixed(j))
                   {
                       if(i != j)
                           hessians(I).write(i, j, Matrix3x3::Zero());
                   }
               })
        .wait();
}

void FEMLinearSubsystem::Impl::_assemble_producers(GlobalLinearSystem::DiagInfo& info)
{
    FiniteElementEnergyProducer::AssemblyInfo assembly_info;
    assembly_info.hessians = info.hessians().subview(energy_producer_hessian_offset,
                                                     energy_producer_hessian_count);
    assembly_info.dt = dt;

    for(auto& producer : fem().energy_producers)
    {
        producer->assemble_gradient_hessian(assembly_info);
    }

    using namespace muda;

    // need to assemble doublet gradient to dense gradient
    const auto& producer_gradients = fem().energy_producer_gradients;
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(producer_gradients.doublet_count(),
               [dst_gradient = info.gradients().viewer().name("dst_gradient"),
                src_gradient = producer_gradients.viewer().name("src_gradient")] __device__(int I) mutable
               {
                   auto&& [i, G3] = src_gradient(I);
                   dst_gradient.segment<3>(i * 3).atomic_add(G3);
               });
}

void FEMLinearSubsystem::Impl::_assemble_dytopo_effect(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;

    if(dytopo_effect_receiver)  //  if dytopo_effect enabled
    {
        auto dytopo_effect_gradient_count =
            dytopo_effect_receiver->graidients().doublet_count();

        // 1) Assemble DyTopoEffect Gradient to Gradient
        if(dytopo_effect_gradient_count)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(dytopo_effect_gradient_count,
                       [dytopo_effect_gradient =
                            dytopo_effect_receiver->graidients().cviewer().name("dytopo_effect_gradient"),
                        gradients = info.gradients().viewer().name("gradients"),
                        vertex_offset = finite_element_vertex_reporter->vertex_offset(),
                        is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
                       {
                           const auto& [g_i, G3] = dytopo_effect_gradient(I);
                           auto i = g_i - vertex_offset;  // from global to local
                           gradients.segment<3>(i * 3).atomic_add(G3);
                       });
        }

        // 2) Assemble DyTopoEffect Hessian to Hessian
        if(dytopo_effect_hessian_count)
        {
            auto dst_H3x3s = info.hessians().subview(dytopo_effect_hessian_offset,
                                                     dytopo_effect_hessian_count);

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(dytopo_effect_hessian_count,
                       [dytopo_effect_hessian =
                            dytopo_effect_receiver->hessians().cviewer().name("dytopo_effect_hessian"),
                        hessians = dst_H3x3s.viewer().name("hessians"),
                        vertex_offset =
                            finite_element_vertex_reporter->vertex_offset()] __device__(int I) mutable
                       {
                           const auto& [g_i, g_j, H3] = dytopo_effect_hessian(I);
                           auto i = g_i - vertex_offset;
                           auto j = g_j - vertex_offset;

                           hessians(I).write(i, j, H3);
                       });
        }
    }
}

void FEMLinearSubsystem::Impl::_assemble_animation(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;
    if(finite_element_animator)
    {
        auto hessians = info.hessians().subview(animator_hessian_offset, animator_hessian_count);
        FiniteElementAnimator::AssembleInfo this_info{info.gradients(), hessians, dt};
        finite_element_animator->assemble(this_info);
    }
}

void FEMLinearSubsystem::Impl::accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    info.statisfied(true);
}

void FEMLinearSubsystem::Impl::retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    using namespace muda;

    auto dxs = fem().dxs.view();
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(fem().xs.size(),
               [dxs = dxs.viewer().name("dxs"),
                result = info.solution().viewer().name("result")] __device__(int i) mutable
               {
                   dxs(i) = -result.segment<3>(i * 3).as_eigen();

                   // cout << "solution dx(" << i << "):" << dxs(i).transpose().eval() << "\n";
               });
}

void FEMLinearSubsystem::do_report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    m_impl.report_extent(info);
}

void FEMLinearSubsystem::do_assemble(GlobalLinearSystem::DiagInfo& info)
{
    m_impl.assemble(info);
}

void FEMLinearSubsystem::do_accuracy_check(GlobalLinearSystem::AccuracyInfo& info)
{
    m_impl.accuracy_check(info);
}

void FEMLinearSubsystem::do_retrieve_solution(GlobalLinearSystem::SolutionInfo& info)
{
    m_impl.retrieve_solution(info);
}

void FEMLinearSubsystem::do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    m_impl.report_init_extent(info);
}

void FEMLinearSubsystem::do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    m_impl.receive_init_dof_info(world(), info);
}

}  // namespace uipc::backend::cuda
