#include <affine_body/abd_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen.h>
#include <utils/matrix_assembler.h>
#include <utils/matrix_unpacker.h>
#include <uipc/builtin/attribute_name.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDLinearSubsystem);

void ABDLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo& info)
{
    m_impl.affine_body_dynamics        = require<AffineBodyDynamics>();
    m_impl.affine_body_vertex_reporter = require<AffineBodyVertexReporter>();

    auto aba = find<AffineBodyAnimator>();
    if(aba)
        m_impl.affine_body_animator = *aba;
    auto contact = find<ABDContactReceiver>();
    if(contact)
        m_impl.abd_contact_receiver = *contact;
}

void ABDLinearSubsystem::Impl::report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    info.extent(abd().body_count() * 12);
}

void ABDLinearSubsystem::Impl::receive_init_dof_info(WorldVisitor& w,
                                                     GlobalLinearSystem::InitDofInfo& info)
{
    auto& geo_infos = abd().geo_infos;
    auto  geo_slots = w.scene().geometries();

    IndexT offset = info.dof_offset();

    // fill the dof_offset and dof_count for each geometry
    affine_body_dynamics->for_each(
        geo_slots,
        [&](const AffineBodyDynamics::ForEachInfo& foreach_info, geometry::SimplicialComplex& sc)
        {
            auto I          = foreach_info.global_index();
            auto dof_offset = sc.meta().find<IndexT>(builtin::dof_offset);
            UIPC_ASSERT(dof_offset, "dof_offset not found on ABD mesh why can it happen?");
            auto dof_count = sc.meta().find<IndexT>(builtin::dof_count);
            UIPC_ASSERT(dof_count, "dof_count not found on ABD mesh why can it happen?");

            IndexT this_dof_count = 12 * sc.instances().size();
            view(*dof_offset)[0]  = offset;
            view(*dof_count)[0]   = this_dof_count;

            offset += this_dof_count;
        });

    UIPC_ASSERT(offset == info.dof_offset() + info.dof_count(), "dof size mismatch");
}

void ABDLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Full,
                "Now only support Full Hessian");

    constexpr SizeT M12x12_to_M3x3 = (12 * 12) / (3 * 3);
    constexpr SizeT G12_to_dof     = 12;

    // 1) Hessian Count
    SizeT H12x12_count = 0;

    // body hessian
    H12x12_count += abd().body_id_to_body_hessian.size();
    if(abd_contact_receiver)
    {
        H12x12_count += contact().contact_hessian.triplet_count();
    }
    if(affine_body_animator)
    {
        AffineBodyAnimator::ExtentInfo anim_extent_info;
        affine_body_animator->report_extent(anim_extent_info);
        H12x12_count += anim_extent_info.hessian_block_count;
    }

    auto H3x3_count = H12x12_count * M12x12_to_M3x3;


    // 2) Gradient Count
    SizeT G12_count = abd().body_count();
    auto  dof_count = abd().abd_body_count * G12_to_dof;

    info.extent(H3x3_count, dof_count);
}

void ABDLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;

    // 1) Kinetic & Shape
    IndexT offset = 0;
    {
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(abd().body_count(),
                   [body_gradient = abd().body_id_to_body_gradient.cviewer().name("abd_gradient"),
                    gradient = info.gradient().viewer().name("gradient")] __device__(int i) mutable
                   { gradient.segment<12>(i * 12) = body_gradient(i); });

        auto body_count = abd().body_id_to_body_hessian.size();
        auto H3x3_count = body_count * 16;
        auto body_H3x3  = info.hessian().subview(offset, H3x3_count);
        ParallelFor()
            .file_line(__FILE__, __LINE__)
            .apply(body_count,
                   [dst = body_H3x3.viewer().name("dst_hessian"),
                    src = abd().body_id_to_body_hessian.cviewer().name("src_hessian"),
                    diag_hessian = abd().diag_hessian.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       TripletMatrixUnpacker MA{dst};

                       MA.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
                           .write(I * 4,          // begin row
                                  I * 4,          // begin col
                                  src(I));

                       diag_hessian(I) = src(I);
                   });

        offset += H3x3_count;
    }

    // 2) Contact
    if(abd_contact_receiver)
    {
        auto vertex_offset = affine_body_vertex_reporter->vertex_offset();

        auto contact_gradient_count = contact().contact_gradient.doublet_count();

        if(contact_gradient_count)
        {

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(contact_gradient_count,
                       [contact_gradient = contact().contact_gradient.cviewer().name("contact_gradient"),
                        gradient = info.gradient().viewer().name("gradient"),
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
                               Vector12 G12 = J_i.T() * G3;
                               gradient.segment<12>(body_i * 12).atomic_add(G12);
                           }
                       });
        }

        auto contact_hessian_count = contact().contact_hessian.triplet_count();
        auto H3x3_count            = contact_hessian_count * 16;
        auto contact_H3x3          = info.hessian().subview(offset, H3x3_count);

        if(contact_hessian_count)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(contact_hessian_count,
                       [contact_hessian = contact().contact_hessian.cviewer().name("contact_hessian"),
                        dst = contact_H3x3.viewer().name("dst_hessian"),
                        v2b = abd().vertex_id_to_body_id.cviewer().name("v2b"),
                        Js  = abd().vertex_id_to_J.cviewer().name("Js"),
                        is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                        diag_hessian = abd().diag_hessian.viewer().name("diag_hessian"),
                        vertex_offset = vertex_offset] __device__(int I) mutable
                       {
                           const auto& [g_i, g_j, H3x3] = contact_hessian(I);

                           auto i = g_i - vertex_offset;
                           auto j = g_j - vertex_offset;

                           auto body_i = v2b(i);
                           auto body_j = v2b(j);

                           auto J_i = Js(i);
                           auto J_j = Js(j);

                           Matrix12x12 H12x12;
                           if(is_fixed(body_i) || is_fixed(body_j))
                           {
                               H12x12.setZero();
                           }
                           else
                           {
                               H12x12 = ABDJacobi::JT_H_J(J_i.T(), H3x3, J_j);

                               // Fill diagonal hessian for diag-inv preconditioner
                               // TODO: Maybe later we can move it to a separate kernel for readability
                               if(body_i == body_j)
                               {
                                   eigen::atomic_add(diag_hessian(body_i), H12x12);
                               }
                           }

                           TripletMatrixUnpacker MU{dst};
                           MU.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
                               .write(body_i * 4,  // begin row
                                      body_j * 4,  // begin col
                                      H12x12);
                       });
        }

        offset += H3x3_count;
    }

    // 3) Animator
    if(affine_body_animator)
    {
        AffineBodyAnimator::AssembleInfo anim_info;
        affine_body_animator->assemble(anim_info);

        auto gradient_count = anim_info.gradients().doublet_count();

        if(gradient_count)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(gradient_count,
                       [animator_gradient = anim_info.gradients().cviewer().name("animator_gradient"),
                        gradient = info.gradient().viewer().name("abd_gradient"),
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
                               gradient.segment<12>(body_i * 12).atomic_add(G12);
                           }
                       });
        }

        auto anim_hessian_count = anim_info.hessians().triplet_count();
        auto H3x3_count         = anim_hessian_count * 16;
        auto anim_H3x3          = info.hessian().subview(offset, H3x3_count);

        if(anim_hessian_count)
        {
            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(anim_hessian_count,
                       [animator_hessian = anim_info.hessians().cviewer().name("animator_hessian"),
                        dst = anim_H3x3.viewer().name("triplet"),
                        is_fixed = abd().body_id_to_is_fixed.cviewer().name("is_fixed"),
                        diag_hessian = abd().diag_hessian.viewer().name(
                            "diag_hessian")] __device__(int I) mutable
                       {
                           TripletMatrixUnpacker MU{dst};

                           const auto& [body_i, body_j, H12x12] = animator_hessian(I);

                           if(is_fixed(body_i) || is_fixed(body_j))
                           {
                               MU.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
                                   .write(body_i * 4,  // begin row
                                          body_j * 4,  // begin col
                                          Matrix12x12::Zero());
                           }
                           else
                           {
                               MU.block<4, 4>(I * 4 * 4)  // triplet range of [I*4*4, (I+1)*4*4)
                                   .write(body_i * 4,  // begin row
                                          body_j * 4,  // begin col
                                          H12x12);

                               if(body_i == body_j)
                               {
                                   eigen::atomic_add(diag_hessian(body_i), H12x12);
                               }
                           }
                       });
        }

        offset += H3x3_count;
    }

    UIPC_ASSERT(offset == info.hessian().triplet_count(),
                "Hessian count mismatch, expect {}, got {}",
                offset,
                info.hessian().triplet_count());
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

void ABDLinearSubsystem::do_report_init_extent(GlobalLinearSystem::InitDofExtentInfo& info)
{
    m_impl.report_init_extent(info);
}

void ABDLinearSubsystem::do_receive_init_dof_info(GlobalLinearSystem::InitDofInfo& info)
{
    m_impl.receive_init_dof_info(world(), info);
}
}  // namespace uipc::backend::cuda
