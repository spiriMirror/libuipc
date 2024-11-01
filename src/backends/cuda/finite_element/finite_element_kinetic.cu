#include <finite_element/finite_element_kinetic.h>
#include <finite_element/finite_element_diff_dof_reporter.h>

namespace uipc::backend::cuda
{
void FiniteElementKinetic::do_build(BuildInfo& info)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
    m_impl.finite_element_method->add_constitution(this);
}

void FiniteElementKinetic::do_report_extent(ReportExtentInfo& info)
{
    auto vert_count = m_impl.fem().xs.size();
    info.energy_count(vert_count);
    info.stencil_dim(1);
}

void FiniteElementKinetic::do_compute_energy(ComputeEnergyInfo& info)
{
    m_impl.compute_energy(info);
}

void FiniteElementKinetic::do_compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    m_impl.compute_gradient_hessian(info);
}

void FiniteElementKinetic::Impl::compute_energy(ComputeEnergyInfo& info)
{
    using namespace muda;
    // Compute kinetic energy
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(fem().xs.size(),
               [is_fixed = fem().is_fixed.cviewer().name("is_fixed"),
                xs       = fem().xs.cviewer().name("xs"),
                x_tildes = fem().x_tildes.viewer().name("x_tildes"),
                masses   = fem().masses.cviewer().name("masses"),
                Ks = info.energies().viewer().name("kinetic_energy")] __device__(int i) mutable
               {
                   auto& K = Ks(i);
                   if(is_fixed(i))
                   {
                       K = 0.0;
                   }
                   else
                   {
                       const Vector3& x       = xs(i);
                       const Vector3& x_tilde = x_tildes(i);
                       Float          M       = masses(i);
                       Vector3        dx      = x - x_tilde;
                       K                      = 0.5 * M * dx.dot(dx);
                   }
               });
}
void FiniteElementKinetic::Impl::compute_gradient_hessian(ComputeGradientHessianInfo& info)
{
    using namespace muda;

    // Kinetic
    ParallelFor()
        .file_line(__FILE__, __LINE__)
        .apply(fem().xs.size(),
               [xs         = fem().xs.viewer().name("xs"),
                x_tildes   = fem().x_tildes.cviewer().name("x_tildes"),
                vs         = fem().vs.viewer().name("vs"),
                masses     = fem().masses.cviewer().name("masses"),
                is_fixed   = fem().is_fixed.cviewer().name("is_fixed"),
                is_dynamic = fem().is_dynamic.cviewer().name("is_dynamic"),
                G3s        = info.gradients().viewer().name("G3s"),
                H3x3s = info.hessians().viewer().name("H3x3s")] __device__(int i) mutable
               {
                   auto& m       = masses(i);
                   auto& x       = xs(i);
                   auto& x_tilde = x_tildes(i);

                   Vector3   G;
                   Matrix3x3 H;

                   if(is_fixed(i))  // fixed
                   {
                       G = Vector3::Zero();
                   }
                   else
                   {
                       G = m * (x - x_tilde);
                   }

                   H = masses(i) * Matrix3x3::Identity();

                   G3s(i).write(i, G);
                   H3x3s(i).write(i, i, H);
               });
}

class FiniteElementKineticDiffDofReporter : public FiniteElementDiffDofReporter
{
    //tex:
    //deal with:
    //
    //$$
    //\begin{bmatrix}
    //\frac{\partial^2 E^{[k]}}{\partial X^{[k]} \partial X^{[k-2]}} &
    //\frac{\partial^2 E^{[k]}}{\partial X^{[k]} \partial X^{[k-1]}}
    //\end{bmatrix}
    //$$
    //
    // Note: $$ \frac{\partial^2 E^{[k]}}{\partial X^{[k]} \partial X^{[k]}} $$
    // is created directly by the global linear system


  public:
    using FiniteElementDiffDofReporter::FiniteElementDiffDofReporter;

    void do_build(BuildInfo& info) override {}

    void do_report_extent(DiffDofExtentInfo& info) override
    {
        SizeT triplet_count = 0;
        auto  F             = info.frame();


        if(F == 2)
        {
            auto Fm1       = F - 1;
            auto dof_count = info.dof_count(Fm1);
            triplet_count  = dof_count;
        }
        else if(F >= 3)
        {
            auto Fm1        = F - 1;
            auto dof_count1 = info.dof_count(Fm1);

            auto Fm2        = F - 2;
            auto dof_count2 = info.dof_count(Fm2);

            triplet_count = dof_count1 + dof_count2;
        }

        info.triplet_count(triplet_count);
    }

    void do_assemble(DiffDofInfo& info) override
    {
        using namespace muda;
        auto F          = info.frame();
        auto row_offset = info.dof_offset(F);
        auto row_count  = info.dof_count(F);

        auto triplet_offset = 0;

        if(F == 2)
        {
            // H^{[k][k-1]} = -M

            auto Fm1 = F - 1;

            auto col_count  = info.dof_count(Fm1);
            auto col_offset = info.dof_offset(Fm1);
            auto N          = col_count / 3;
            UIPC_ASSERT(row_count == col_count, "Now we don't support variant dof across frames");

            auto H = info.H();
            auto H_k_k_1 =
                H.subview(triplet_offset, col_count)
                    .submatrix({row_offset, col_offset}, {row_count, col_count});

            ParallelFor()
                .file_line(__FILE__, __LINE__)
                .apply(N,
                       [H_k_k_1 = H_k_k_1.viewer().name("H_k_k_1"),
                        masses = fem().masses.cviewer().name("masses")] __device__(int I) mutable
                       {
                           //tex:
                           //$$
                           //G^{[2]} = m (x^{[2]} - 2x^{[1]} + x^{[0]} + g \cdot dt^2)
                           //$$
                           //$$ H^{[2][1]} = -2m $$

                           auto m = masses(I);

                           for(int i = 0; i < 3; ++i)
                               H_k_k_1(I * 3 + i).write(I * 3 + i, I * 3 + i, -2 * m);
                       });

            triplet_offset = col_count;
        }
        else if(F >= 3)
        {
            // H^{[k][k-1]} = -2M
            {
                auto Fm1 = F - 1;

                auto col_count  = info.dof_count(Fm1);
                auto col_offset = info.dof_offset(Fm1);
                auto N          = col_count / 3;

                UIPC_ASSERT(row_count == col_count,
                            "Now we don't support variant dof across frames");

                auto H = info.H();
                auto H_k_k_1 =
                    H.subview(triplet_offset, col_count)
                        .submatrix({row_offset, col_offset}, {row_count, col_count});

                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(N,
                           [H_k_k_1 = H_k_k_1.viewer().name("H_k_k_1"),
                            masses = fem().masses.cviewer().name("masses")] __device__(int I) mutable
                           {
                               //tex:
                               //$$
                               //G^{[k]} = m (x^{[k]} - 2x^{[k-1]} + x^{[k-2]} + g \cdot dt^2)
                               //$$
                               //$$ H^{[k][k-1]} = -2m $$

                               auto m = masses(I);

                               for(int i = 0; i < 3; ++i)
                                   H_k_k_1(I * 3 + i).write(I * 3 + i, I * 3 + i, -2.0 * m);
                           });

                triplet_offset += col_count;
            }


            // H^{[k][k-2]} = M
            {
                auto Fm2 = F - 2;

                auto col_count  = fem().dof_count(Fm2);
                auto col_offset = fem().dof_offset(Fm2);
                auto N          = col_count / 3;

                UIPC_ASSERT(row_count == col_count,
                            "Now we don't support variant dof across frames");

                auto H = info.H();
                auto H_k_k_2 =
                    H.subview(triplet_offset, col_count)
                        .submatrix({row_offset, col_offset}, {row_count, col_count});

                ParallelFor()
                    .file_line(__FILE__, __LINE__)
                    .apply(N,
                           [H_k_k_2 = H_k_k_2.viewer().name("H_k_k_2"),
                            masses = fem().masses.cviewer().name("masses")] __device__(int I) mutable
                           {
                               //tex:
                               //$$
                               //G^{[k]} = m (x^{[k]} - 2x^{[k-1]} + x^{[k-2]} + g \cdot dt^2)
                               //$$
                               //$$ H^{[k][k-2]} = m $$

                               auto m = masses(I);

                               for(int i = 0; i < 3; ++i)
                                   H_k_k_2(I * 3 + i).write(I * 3 + i, I * 3 + i, m);
                           });

                triplet_offset += col_count;
            }
        }

        UIPC_ASSERT(triplet_offset == info.H().triplet_count(), "triplet count mismatch");
    }
};

REGISTER_SIM_SYSTEM(FiniteElementKinetic);

REGISTER_SIM_SYSTEM(FiniteElementKineticDiffDofReporter);
}  // namespace uipc::backend::cuda
