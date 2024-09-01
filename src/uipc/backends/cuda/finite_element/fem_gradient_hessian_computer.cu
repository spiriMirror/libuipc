#include <finite_element/fem_gradient_hessian_computer.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
// constitutions
#include <finite_element/codim_0d_constitution.h>
#include <finite_element/codim_1d_constitution.h>
#include <finite_element/codim_2d_constitution.h>
#include <finite_element/fem_3d_constitution.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMGradientHessianComputer);

void FEMGradientHessianComputer::do_build()
{
    m_impl.finite_element_method    = &require<FiniteElementMethod>();
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
        .kernel_name("Kinetic Energy")
        .apply(fem().xs.size(),
               [xs       = fem().xs.viewer().name("xs"),
                x_tildes = fem().x_tildes.viewer().name("x_tildes"),
                vs       = fem().vs.viewer().name("vs"),
                masses   = fem().masses.viewer().name("masses"),
                is_fixed = fem().is_fixed.viewer().name("is_fixed"),
                vertex_kinetic_energies =
                    fem().vertex_kinetic_energies.viewer().name("vertex_kinetic_energies"),
                G3s = fem().G3s.viewer().name("G3s"),
                H3x3s = fem().H3x3s.viewer().name("H3x3s")] __device__(int i) mutable
               {
                   auto& m       = masses(i);
                   auto& x       = xs(i);
                   auto& x_tilde = x_tildes(i);

                   Vector3   G = Vector3::Zero();
                   Matrix3x3 H = masses(i) * Matrix3x3::Identity();

                   if(is_fixed(i))
                   {
                       // G = Vector3::Zero();
                   }
                   else
                   {
                       G = m * (x - x_tilde);
                   }

                   G3s(i)   = G;
                   H3x3s(i) = H;

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
}
}  // namespace uipc::backend::cuda
