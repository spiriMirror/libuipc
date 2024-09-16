#include <affine_body/abd_gradient_hessian_computer.h>
#include <affine_body/affine_body_constitution.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(ABDGradientHessianComputer);

void ABDGradientHessianComputer::do_build()
{
    m_impl.affine_body_dynamics = &require<AffineBodyDynamics>();
    m_impl.affine_body_animator = find<AffineBodyAnimator>();

    auto& gradient_hessian_computer = require<GradientHessianComputer>();
    // Register the action to compute the gradient and hessian
    gradient_hessian_computer.on_compute_gradient_hessian(
        *this,
        [this](GradientHessianComputer::ComputeInfo& info)
        { m_impl.compute_gradient_hessian(info); });
}

void ABDGradientHessianComputer::Impl::compute_gradient_hessian(GradientHessianComputer::ComputeInfo& info)
{
    using namespace muda;

    auto& body_id_to_body_hessian   = abd().body_id_to_body_hessian;
    auto& body_id_to_body_gradient  = abd().body_id_to_body_gradient;
    auto& body_id_to_is_fixed       = abd().body_id_to_is_fixed;
    auto& body_id_to_q              = abd().body_id_to_q;
    auto& body_id_to_q_tilde        = abd().body_id_to_q_tilde;
    auto& body_id_to_abd_mass       = abd().body_id_to_abd_mass;
    auto& body_id_to_kinetic_energy = abd().body_id_to_kinetic_energy;
    auto& constitutions             = abd().constitutions;
    auto& diag_hessian              = abd().diag_hessian;


    auto async_fill = []<typename T>(muda::DeviceBuffer<T>& buf, const T& value)
    { muda::BufferLaunch().fill<T>(buf.view(), value); };

    async_fill(body_id_to_body_hessian, Matrix12x12::Zero().eval());
    async_fill(body_id_to_body_gradient, Vector12::Zero().eval());

    // 1) compute all shape gradients and hessians
    for(auto&& [i, cst] : enumerate(constitutions.view()))
    {
        const auto& constitution_info = abd().constitution_infos[i];
        auto        body_offset       = constitution_info.body_offset;
        auto        body_count        = constitution_info.body_count;

        if(body_count == 0)
            continue;

        AffineBodyDynamics::ComputeGradientHessianInfo this_info{
            &abd(),
            i,
            body_id_to_body_gradient.view(body_offset, body_count),
            body_id_to_body_hessian.view(body_offset, body_count),
            info.dt()};
        cst->compute_gradient_hessian(this_info);
    }

    // 2) add kinetic energy gradient and hessian
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(abd().abd_body_count,
               [is_fixed = body_id_to_is_fixed.cviewer().name("is_fixed"),
                qs       = body_id_to_q.cviewer().name("qs"),
                q_tildes = body_id_to_q_tilde.cviewer().name("q_tildes"),
                masses   = body_id_to_abd_mass.cviewer().name("masses"),
                Ks = body_id_to_kinetic_energy.cviewer().name("kinetic_energy"),
                diag_hessian = diag_hessian.viewer().name("diag_hessian"),
                m_hessians = body_id_to_body_hessian.viewer().name("hessians"),
                gradients = body_id_to_body_gradient.viewer().name("gradients")] __device__(int i) mutable
               {
                   const auto& q = qs(i);
                   auto&       H = m_hessians(i);
                   auto&       G = gradients(i);
                   const auto& M = masses(i);

                   // cout << "q(" << i << ")=" << q.transpose().eval() << "\n";

                   if(is_fixed(i))
                   {
                       H = M.to_mat();
                       G = Vector12::Zero();
                   }
                   else
                   {
                       const auto& q_tilde = q_tildes(i);

                       const auto& K = Ks(i);
                       G += M * (q - q_tilde);
                       H += M.to_mat();

                       Vector9   eigen_values;
                       Matrix9x9 eigen_vectors;
                       Matrix9x9 mat9 = H.block<9, 9>(3, 3);
                       muda::eigen::evd(mat9, eigen_values, eigen_vectors);
                       for(int i = 0; i < 9; ++i)
                       {
                           auto& v = eigen_values(i);
                           v       = v < 0.0 ? 0.0 : v;
                       }
                       H.block<9, 9>(3, 3) = eigen_vectors * eigen_values.asDiagonal()
                                             * eigen_vectors.transpose();
                   }
               });

    // 3) compute animator gradient and hessian
    if(affine_body_animator)
    {
        affine_body_animator->compute_gradient_hessian(info);
    }
}

AffineBodyDynamics::Impl& ABDGradientHessianComputer::Impl::abd() noexcept
{
    return affine_body_dynamics->m_impl;
}
}  // namespace uipc::backend::cuda
