#include <affine_body/abd_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDLinearSubsystem);

void ABDLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo& info)
{
    m_impl.reduce_hessian = false;  // default false, because of low convert efficiency according to profiling

    m_impl.affine_body_dynamics        = require<AffineBodyDynamics>();
    m_impl.affine_body_vertex_reporter = require<AffineBodyVertexReporter>();

    auto aba = find<AffineBodyAnimator>();
    if(aba)
        m_impl.affine_body_animator = *aba;
    auto contact = find<ABDContactReceiver>();
    if(contact)
        m_impl.abd_contact_receiver = *contact;
}

void ABDLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Full,
                "Now only support Full Hessian");

    constexpr SizeT M12x12_to_M3x3 = (12 * 12) / (3 * 3);
    constexpr SizeT G12_to_dof     = 12;

    assemble_bodies();

    // 1) Hessian Count
    SizeT H12x12_count        = A_view.triplet_count();
    auto  hessian_block_count = H12x12_count * M12x12_to_M3x3;


    // 2) Gradient Count
    SizeT G12_count = abd().body_count();
    auto  dof_count = abd().abd_body_count * G12_to_dof;

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

    // body gradient
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(abd().body_count(),
               [abd_gradient = abd().body_id_to_body_gradient.cviewer().name("abd_gradient"),
                gradient = info.gradient().viewer().name("gradient")] __device__(int i) mutable
               { gradient.segment<12>(i * 12) = abd_gradient(i); });
}

void ABDLinearSubsystem::Impl::_assemble_hessian(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;

    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(A_view.triplet_count(),
               [dst = info.hessian().viewer().name("dst_hessian"),
                src = A_view.cviewer().name("src_hessian")] __device__(int I) mutable
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

void ABDLinearSubsystem::Impl::assemble_bodies()
{
    using namespace muda;

    auto H12x12_count = abd().body_id_to_body_hessian.size();

    if(abd_contact_receiver)
        H12x12_count += contact().contact_hessian.triplet_count();

    if(affine_body_animator)
    {
        AffineBodyAnimator::ExtentInfo info;
        affine_body_animator->report_extent(info);
        H12x12_count += info.hessian_block_count;
    }

    auto async_fill = []<typename T>(muda::DeviceBuffer<T>& buf, const T& value)
    { muda::BufferLaunch().fill<T>(buf.view(), value); };

    async_fill(abd().diag_hessian, Matrix12x12::Zero().eval());

    if(triplet_A.triplet_capacity() < H12x12_count)
    {
        auto reserve_count = H12x12_count * reserve_ratio;
        triplet_A.reserve_triplets(reserve_count);

        if(reduce_hessian)
            bcoo_A.reserve_triplets(reserve_count);
    }

    triplet_A.resize_triplets(H12x12_count);
    triplet_A.reshape(abd().abd_body_count, abd().abd_body_count);

    auto A = triplet_A.view();

    // body hessian
    auto offset = 0;
    {
        auto count = abd().body_id_to_body_hessian.size();
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(count,
                   [body_hessian = abd().body_id_to_body_hessian.cviewer().name("body_hessian"),
                    triplet = A.subview(offset, count).viewer().name("triplet"),
                    diag_hessian = abd().diag_hessian.viewer().name(
                        "diag_hessian")] __device__(int i) mutable
                   {
                       triplet(i).write(i, i, body_hessian(i));
                       diag_hessian(i) += body_hessian(i);
                   });

        offset += count;
    }

    // contact gradient and hessian
    if(abd_contact_receiver)  // if contact is enabled
    {
        auto vertex_offset = affine_body_vertex_reporter->vertex_offset();

        auto gradient_count = contact().contact_gradient.doublet_count();
        if(gradient_count)
        {
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(gradient_count,
                       [contact_gradient = contact().contact_gradient.cviewer().name("contact_gradient"),
                        abd_gradient = abd().body_id_to_body_gradient.viewer().name("abd_gradient"),
                        v2b = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                        Js  = abd().vertex_id_to_J.cviewer().name("Js"),
                        is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                        vertex_offset = vertex_offset] __device__(int I) mutable
                       {
                           const auto& [g_i, G3] = contact_gradient(I);

                           auto i      = g_i - vertex_offset;
                           auto body_i = v2b(i);
                           auto J_i    = Js(i);

                           if(is_fixed(body_i))
                           {
                               // Do nothing
                           }
                           else
                           {
                               Vector12 G12  = J_i.T() * G3;
                               auto&    dstG = abd_gradient(body_i);
                               muda::eigen::atomic_add(dstG, G12);
                           }
                       });
        }

        auto hessian_count = contact().contact_hessian.triplet_count();

        if(hessian_count)
        {
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(hessian_count,
                       [contact_hessian = contact().contact_hessian.cviewer().name("contact_hessian"),
                        triplet = A.subview(offset, hessian_count).viewer().name("triplet"),
                        vertex_offset = vertex_offset,
                        v2b = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                        Js  = abd().vertex_id_to_J.cviewer().name("Js"),
                        is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                        diag_hessian = abd().diag_hessian.viewer().name(
                            "diag_hessian")] __device__(int I) mutable
                       {
                           const auto& [g_i, g_j, H3x3] = contact_hessian(I);

                           auto i = g_i - vertex_offset;
                           auto j = g_j - vertex_offset;

                           auto body_i = v2b(i);
                           auto body_j = v2b(j);

                           //out << "body_i=" << body_i << " body_j=" << body_j << "\n";

                           auto J_i = Js(i);
                           auto J_j = Js(j);

                           if(is_fixed(body_i) || is_fixed(body_j))
                           {
                               triplet(I).write(body_i, body_j, Matrix12x12::Zero());
                           }
                           else
                           {
                               Matrix12x12 H12x12 = ABDJacobi::JT_H_J(J_i.T(), H3x3, J_j);
                               triplet(I).write(body_i, body_j, H12x12);
                               // cout << "H12x12: \n" << H12x12 << "\n";

                               if(body_i == body_j)
                               {
                                   eigen::atomic_add(diag_hessian(body_i), H12x12);
                               }
                           }
                       });
        }

        offset += hessian_count;
    }

    // animator gradient and hessian
    if(affine_body_animator)
    {
        AffineBodyAnimator::AssembleInfo info;
        affine_body_animator->assemble(info);

        auto gradient_count = info.gradients().doublet_count();
        if(gradient_count)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(gradient_count,
                       [animator_gradient = info.gradients().cviewer().name("animator_gradient"),
                        abd_gradient = abd().body_id_to_body_gradient.viewer().name("abd_gradient"),
                        is_fixed = abd().body_id_to_is_fixed.cviewer().name(
                            "is_fixed")] __device__(int I) mutable
                       {
                           const auto& [body_i, G12] = animator_gradient(I);

                           if(is_fixed(body_i))
                           {
                               // Do nothing
                           }
                           else
                           {
                               muda::eigen::atomic_add(abd_gradient(body_i), G12);
                           }
                       });
        }

        auto hessian_count = info.hessians().triplet_count();
        if(hessian_count)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(hessian_count,
                       [animator_hessian = info.hessians().cviewer().name("animator_hessian"),
                        triplet = A.subview(offset, hessian_count).viewer().name("triplet"),
                        is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                        diag_hessian = abd().diag_hessian.viewer().name(
                            "diag_hessian")] __device__(int I) mutable
                       {
                           const auto& [body_i, body_j, H12x12] = animator_hessian(I);

                           if(is_fixed(body_i) || is_fixed(body_j))
                           {
                               triplet(I).write(body_i, body_j, Matrix12x12::Zero());
                           }
                           else
                           {
                               triplet(I).write(body_i, body_j, H12x12);

                               if(body_i == body_j)
                               {
                                   eigen::atomic_add(diag_hessian(body_i), H12x12);
                               }
                           }
                       });
        }
    }

    if(reduce_hessian)
    {
        converter.convert(triplet_A, bcoo_A);
        A_view = bcoo_A.view();
    }
    else
    {
        A_view = triplet_A.view();
    }
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
                   // cout << "solution dq("<< i << "):" << dq(i).transpose().eval() << "\n";
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
