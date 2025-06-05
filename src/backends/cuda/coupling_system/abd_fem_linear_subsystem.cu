#include <linear_system/off_diag_linear_subsystem.h>
#include <coupling_system/abd_fem_contact_receiver.h>
#include <coupling_system/fem_abd_contact_receiver.h>
#include <affine_body/abd_linear_subsystem.h>
#include <finite_element/fem_linear_subsystem.h>
#include <linear_system/global_linear_system.h>
#include <affine_body/affine_body_dynamics.h>
#include <finite_element/finite_element_method.h>
#include <affine_body/affine_body_vertex_reporter.h>
#include <finite_element/finite_element_vertex_reporter.h>
#include <kernel_cout.h>

namespace uipc::backend::cuda
{
class ABDFEMLinearSubsystem final : public OffDiagLinearSubsystem
{
  public:
    using OffDiagLinearSubsystem::OffDiagLinearSubsystem;

    SimSystemSlot<GlobalLinearSystem> global_linear_system;

    SimSystemSlot<ABDFEMContactReceiver> abd_fem_contact_receiver;
    SimSystemSlot<FEMABDContactReceiver> fem_abd_contact_receiver;

    SimSystemSlot<ABDLinearSubsystem> abd_linear_subsystem;
    SimSystemSlot<FEMLinearSubsystem> fem_linear_subsystem;

    SimSystemSlot<AffineBodyDynamics>  affine_body_dynamics;
    SimSystemSlot<FiniteElementMethod> finite_element_method;

    SimSystemSlot<AffineBodyVertexReporter>    affine_body_vertex_reporter;
    SimSystemSlot<FiniteElementVertexReporter> finite_element_vertex_reporter;

    virtual void do_build(BuildInfo& info) override
    {
        global_linear_system = require<GlobalLinearSystem>();

        abd_fem_contact_receiver = require<ABDFEMContactReceiver>();
        fem_abd_contact_receiver = require<FEMABDContactReceiver>();

        abd_linear_subsystem = require<ABDLinearSubsystem>();
        fem_linear_subsystem = require<FEMLinearSubsystem>();

        affine_body_dynamics  = require<AffineBodyDynamics>();
        finite_element_method = require<FiniteElementMethod>();

        affine_body_vertex_reporter    = require<AffineBodyVertexReporter>();
        finite_element_vertex_reporter = require<FiniteElementVertexReporter>();

        info.connect(abd_linear_subsystem.view(), fem_linear_subsystem.view());
    }

    virtual void report_extent(GlobalLinearSystem::OffDiagExtentInfo& info) override
    {
        if(!abd_fem_contact_receiver || !fem_abd_contact_receiver)
        {
            info.extent(0, 0);
            return;
        }

        // ABD-FEM Hessian: H12x3
        auto abd_fem_contact_count =
            abd_fem_contact_receiver->contact_hessian().triplet_count();
        auto abd_fem_H3x3_count = abd_fem_contact_count * 4;
        // FEM-ABD Hessian: H3x12
        auto fem_abd_contact_count =
            fem_abd_contact_receiver->contact_hessian().triplet_count();
        auto fem_abd_H3x3_count = fem_abd_contact_count * 4;

        info.extent(abd_fem_H3x3_count, fem_abd_H3x3_count);
    }

    virtual void assemble(GlobalLinearSystem::OffDiagInfo& info) override
    {
        using namespace muda;

        auto count = fem_abd_contact_receiver->contact_hessian().triplet_count();

        UIPC_ASSERT(fem_abd_contact_receiver->contact_hessian().triplet_count()
                        == abd_fem_contact_receiver->contact_hessian().triplet_count(),
                    "ABDFEMLinearSubsystem: contact count mismatch");

        if(count > 0)
        {
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(
                    count,
                    [v2b = affine_body_dynamics->v2b().viewer().name("v2b"),
                     Js  = affine_body_dynamics->Js().viewer().name("Js"),
                     body_is_fixed =
                         affine_body_dynamics->body_is_fixed().viewer().name("body_is_fixed"),
                     vertex_is_fixed =
                         finite_element_method->is_fixed().viewer().name("vertex_is_fixed"),
                     L = info.lr_hessian().viewer().name("L"),
                     R = info.rl_hessian().viewer().name("R"),
                     abd_fem_contact =
                         abd_fem_contact_receiver->contact_hessian().viewer().name("abd_fem_contact"),
                     fem_abd_contact =
                         fem_abd_contact_receiver->contact_hessian().viewer().name("fem_abd_contact"),
                     abd_point_offset = affine_body_vertex_reporter->vertex_offset(),
                     fem_point_offset =
                         finite_element_vertex_reporter->vertex_offset()] __device__(int I) mutable
                    {
                        // 1. ABD-FEM
                        {
                            // global vertex indices
                            auto&& [gI_abd_v, gJ_fem_v, H3x3] = abd_fem_contact(I);

                            // cout << "gI_abd:" << gI_abd_v << " gJ_fem:" << gJ_fem_v << "\n";

                            auto I_abd_v = gI_abd_v - abd_point_offset;
                            auto J_fem_v = gJ_fem_v - fem_point_offset;

                            auto body_id = v2b(I_abd_v);

                            //tex: $\mathbf{J}_{3\times 12}$
                            ABDJacobi J = Js(I_abd_v);

                            auto I4 = 4 * I;

                            Matrix12x3 H = J.to_mat().transpose() * H3x3;

                            if(body_is_fixed(body_id) || vertex_is_fixed(J_fem_v))
                                H.setZero();

                            for(int k = 0; k < 4; ++k)
                            {
                                L(I4 + k).write(4 * body_id + k,  // abd
                                                J_fem_v,          // fem
                                                H.block<3, 3>(3 * k, 0));
                            }
                        }
                        // 2. FEM-ABD
                        {
                            // global vertex indices
                            auto&& [gI_fem_v, gJ_abd_v, H3x3] = fem_abd_contact(I);

                            // cout << "gI_fem:" << gI_fem_v << " gJ_abd:" << gJ_abd_v << "\n";

                            auto I_fem_v = gI_fem_v - fem_point_offset;
                            auto J_abd_v = gJ_abd_v - abd_point_offset;

                            auto body_id = v2b(J_abd_v);

                            //tex: $\mathbf{J}_{3\times 12}$
                            ABDJacobi J = Js(J_abd_v);

                            auto I4 = 4 * I;

                            Matrix3x12 H = H3x3 * J.to_mat();

                            if(body_is_fixed(body_id) || vertex_is_fixed(I_fem_v))
                                H.setZero();

                            for(int k = 0; k < 4; ++k)
                            {
                                R(I4 + k).write(I_fem_v,          // fem
                                                4 * body_id + k,  // abd
                                                H.block<3, 3>(0, 3 * k));
                            }
                        }
                    });
        }
    }
};

REGISTER_SIM_SYSTEM(ABDFEMLinearSubsystem);
}  // namespace uipc::backend::cuda
