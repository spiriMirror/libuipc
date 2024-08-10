#include <finite_element/constitutions/stable_neo_hookean_3d.h>
#include <uipc/common/zip.h>
#include <finite_element/constitutions/stable_neo_hookean_3d_function.h>
#include <finite_element/constitutions/arap_function.h>
#include <finite_element/fem_utils.h>
#include <kernel_cout.h>
#include <muda/ext/eigen/log_proxy.h>
#include <Eigen/Dense>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(StableNeoHookean3D);

U64 StableNeoHookean3D::get_constitution_uid() const
{
    return ConstitutionUID;
}

void StableNeoHookean3D::do_build(BuildInfo& info) {}

void StableNeoHookean3D::do_retrieve(FiniteElementMethod::FEM3DFilteredInfo& info)
{
    m_impl.retrieve(world(), info);
}

void StableNeoHookean3D::do_compute_energy(ComputeEnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void StableNeoHookean3D::do_compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    m_impl.compute_gradient_hessian(info);
}

void StableNeoHookean3D::Impl::retrieve(WorldVisitor& world,
                                        FiniteElementMethod::FEM3DFilteredInfo& info)
{
    auto geo_slots = world.scene().geometries();

    auto N = info.primitive_count();

    h_mus.resize(N);
    h_lambdas.resize(N);

    info.for_each(
        geo_slots,
        [](geometry::SimplicialComplex& sc) -> auto
        {
            auto mu     = sc.tetrahedra().find<Float>("mu");
            auto lambda = sc.tetrahedra().find<Float>("lambda");

            return zip(mu->view(), lambda->view());
        },
        [&](SizeT I, auto mu_and_lambda)
        {
            auto&& [mu, lambda] = mu_and_lambda;
            h_mus[I]            = mu;
            h_lambdas[I]        = lambda;
        });

    mus.resize(N);
    mus.view().copy_from(h_mus.data());

    lambdas.resize(N);
    lambdas.view().copy_from(h_lambdas.data());
}

void StableNeoHookean3D::Impl::compute_energy(ComputeEnergyInfo& info)
{
    using namespace muda;
    using namespace sym::stable_neo_hookean_3d;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.indices().size(),
               [mus     = mus.cviewer().name("mus"),
                lambdas = lambdas.cviewer().name("lambdas"),
                element_energies = info.element_energies().viewer().name("energies"),
                indices = info.indices().viewer().name("indices"),
                xs      = info.xs().viewer().name("xs"),
                Dm_invs = info.Dm_invs().viewer().name("Dm_invs"),
                volumes = info.rest_volumes().viewer().name("volumes"),
                dt      = info.dt()] __device__(int I)
               {
                   const Vector4i&  tet    = indices(I);
                   const Matrix3x3& Dm_inv = Dm_invs(I);
                   Float            mu     = mus(I);
                   Float            lambda = lambdas(I);

                   const Vector3& x0 = xs(tet(0));
                   const Vector3& x1 = xs(tet(1));
                   const Vector3& x2 = xs(tet(2));
                   const Vector3& x3 = xs(tet(3));

                   auto F    = fem::F(x0, x1, x2, x3, Dm_inv);
                   auto VecF = flatten(F);

                   Float E;

                   SNK(E, mu, lambda, VecF);
                   E *= dt * dt;


                   //auto v = volumes(I);
                   //sym::arap::E(E, lambda * dt * dt, v, F);
                   element_energies(I) = E;

                   // element_energies(I) = 0;

                   // cout << "v=" << v << "dt=" << dt << "E=" << E << "\n";
               });
}

void StableNeoHookean3D::Impl::compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    using namespace muda;
    using namespace sym::stable_neo_hookean_3d;

    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(info.indices().size(),
               [mus     = mus.cviewer().name("mus"),
                lambdas = lambdas.cviewer().name("lambdas"),
                indices = info.indices().viewer().name("indices"),
                xs      = info.xs().viewer().name("xs"),
                Dm_invs = info.Dm_invs().viewer().name("Dm_invs"),
                G12s    = info.gradient().viewer().name("gradient"),
                H12x12s = info.hessian().viewer().name("hessian"),
                volumes = info.rest_volumes().viewer().name("volumes"),
                dt      = info.dt()] __device__(int I)
               {
                   const Vector4i&  tet    = indices(I);
                   const Matrix3x3& Dm_inv = Dm_invs(I);
                   Float            mu     = mus(I);
                   Float            lambda = lambdas(I);

                   const Vector3& x0 = xs(tet(0));
                   const Vector3& x1 = xs(tet(1));
                   const Vector3& x2 = xs(tet(2));
                   const Vector3& x3 = xs(tet(3));

                   auto F = fem::F(x0, x1, x2, x3, Dm_inv);

                   cout << "F.det() = " << F.determinant() << "\n";

                   auto VecF = flatten(F);

                   auto dt2 = dt * dt;

                   auto v = volumes(I);

                   Vector9   dEdF;
                   Matrix9x9 ddEddF;
                   //sym::arap::dEdq(dEdF, lambda * dt2, v, F);
                   //sym::arap::ddEddq(ddEddF, lambda * dt2, v, F);

                   dSNKdVecF(dEdF, mu, lambda, VecF);
                   dEdF *= dt2;
                   ddSNKddVecF(ddEddF, mu, lambda, VecF);
                   ddEddF *= dt2;

                   Matrix9x12 dFdx = fem::dFdx(Dm_inv);

                   G12s(I)    = dFdx.transpose() * dEdF;
                   H12x12s(I) = dFdx.transpose() * ddEddF * dFdx;

                   //cout << "G12:" << G12s(I).transpose().eval() << "\n";
                   //cout << "H12x12:\n" << H12x12s(I).eval() << "\n";
                   //G12s(I)    = Vector12::Zero();
                   //H12x12s(I) = Matrix12x12::Zero();
               });
}
}  // namespace uipc::backend::cuda
