#include <finite_element/fem_gradient_hessian_computer.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>

// constitutions
#include <finite_element/codim_0d_constitution.h>
#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_2d_constitution.h>
#include <finite_element/fem_3d_constitution.h>
#include <finite_element/finite_element_extra_constitution.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMGradientHessianComputer);

void FEMGradientHessianComputer::do_build()
{
    m_impl.finite_element_method = require<FiniteElementMethod>();

    auto animator = find<FiniteElementAnimator>();
    if(animator)
        m_impl.finite_element_animator = *animator;

    auto& gradient_hessian_computer = require<GradientHessianComputer>();
    gradient_hessian_computer.on_compute_gradient_hessian(
        *this,
        [this](GradientHessianComputer::ComputeInfo& info)
        { m_impl.compute_gradient_and_hessian(info); });
}

void FEMGradientHessianComputer::Impl::compute_gradient_and_hessian(GradientHessianComputer::ComputeInfo& info)
{
    using namespace muda;

    // Kinetic
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(fem().xs.size(),
               [xs       = fem().xs.viewer().name("xs"),
                x_tildes = fem().x_tildes.cviewer().name("x_tildes"),
                vs       = fem().vs.viewer().name("vs"),
                masses   = fem().masses.cviewer().name("masses"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed"),
                is_kinematic = fem().is_kinematic.cviewer().name("is_kinematic"),
                vertex_kinetic_energies =
                    fem().vertex_kinetic_energies.viewer().name("vertex_kinetic_energies"),
                G3s = fem().G3s.viewer().name("G3s"),
                H3x3s = fem().H3x3s.viewer().name("H3x3s")] __device__(int i) mutable
               {
                   auto& m       = masses(i);
                   auto& x       = xs(i);
                   auto& x_tilde = x_tildes(i);

                   Vector3&   G = G3s(i);
                   Matrix3x3& H = H3x3s(i);

                   if(is_fixed(i))  // fixed
                   {
                       G = Vector3::Zero();
                       H = masses(i) * Matrix3x3::Identity();
                   }
                   else if(is_kinematic(i))  // constraint fully controls the motion
                   {
                       G = Vector3::Zero();
                       H = Matrix3x3::Zero();
                   }
                   else  // free, compute kinetic
                   {
                       G = m * (x - x_tilde);
                       H = masses(i) * Matrix3x3::Identity();
                   }

                   // cout << "Kinetic G:" << G.transpose().eval() << "\n";
               });

    // Elastic
    FiniteElementMethod::ComputeGradientHessianInfo this_info{info.dt()};

    // Codim1D
    for(auto&& [i, cst] : enumerate(fem().codim_1d_constitutions))
    {
        cst->compute_gradient_hessian(this_info);
    }

    // Codim2D
    for(auto&& [i, cst] : enumerate(fem().codim_2d_constitutions))
    {
        cst->compute_gradient_hessian(this_info);
    }

    // FEM3D
    for(auto&& [i, cst] : enumerate(fem().fem_3d_constitutions))
    {
        cst->compute_gradient_hessian(this_info);
    }

    // Extra
    FiniteElementMethod::ComputeExtraGradientHessianInfo extra_info{info.dt()};
    for(auto&& [i, cst] : enumerate(fem().extra_constitutions.view()))
    {
        cst->compute_gradient_hessian(extra_info);
    }

    // Animation
    if(finite_element_animator)
    {
        finite_element_animator->compute_gradient_hessian(info);
    }
}
}  // namespace uipc::backend::cuda
