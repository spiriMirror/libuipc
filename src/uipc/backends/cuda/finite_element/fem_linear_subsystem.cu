#include <finite_element/fem_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen.h>
#include <muda/ext/eigen/evd.h>
#include <muda/ext/eigen/atomic.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMLinearSubsystem);

void FEMLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo&)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
    m_impl.finite_element_vertex_reporter = &require<FiniteElementVertexReporter>();

    m_impl.fem_contact_receiver    = find<FEMContactReceiver>();
    m_impl.finite_element_animator = find<FiniteElementAnimator>();

    m_impl.converter.reserve_ratio(1.1);
}

void FEMLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Full,
                "Now only support Full Hessian");

    // 1) Hessian Count
    auto hessian_block_count =
        fem().H12x12s.size() * H12x12_to_H3x3
        + fem().H9x9s.size() * H9x9_to_H3x3 + fem().H6x6s.size() * H6x6_to_H3x3
        + fem().H3x3s.size() + fem().extra_constitution_hessian.triplet_count();

    if(fem_contact_receiver)  // if contact enabled
    {
        auto contact_count = contact().contact_hessian.triplet_count();
        hessian_block_count += contact_count;
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
    _assemble_gradient(info);
    _assemble_hessian(info);
    _assemble_animation(info);
}

namespace detail
{
    template <int N>
    __device__ void fill_gradient(muda::DenseVectorViewer<Float>& g,
                                  const Vector<Float, 3 * N>&     G3N,
                                  const Vector<IndexT, N>         indices,
                                  muda::CDense1D<IndexT>&         is_fixed)
        requires(N >= 2)
    {
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            int dst = indices(i);
            if(is_fixed(dst))
                continue;
            Vector3 G = G3N.template segment<3>(i * 3);
            g.template segment<3>(dst * 3).atomic_add(G);
        }
    }

    template <int N>
    __device__ void fill_hessian(IndexT                               I,
                                 muda::TripletMatrixViewer<Float, 3>& H3x3,
                                 const Matrix<Float, 3 * N, 3 * N>&   H3Nx3N,
                                 const Vector<IndexT, N>              indices,
                                 muda::CDense1D<IndexT>&              is_fixed)
        requires(N >= 2)
    {
        SizeT offset = I * (N * N);
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
#pragma unroll
            for(int j = 0; j < N; ++j)
            {
                Matrix3x3 H = H3Nx3N.template block<3, 3>(i * 3, j * 3);
                int       L = indices(i);
                int       R = indices(j);

                if(is_fixed(L) || is_fixed(R))
                    H.setZero();

                H3x3(offset++).write(L, R, H);
            }
        }
    }

    template <int N>
    __device__ void fill_diag_hessian(muda::Dense1D<Matrix3x3>&          diag,
                                      const Matrix<Float, 3 * N, 3 * N>& H3Nx3N,
                                      const Vector<IndexT, N> indices,
                                      muda::CDense1D<IndexT>& is_fixed)
        requires(N >= 2)
    {
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto      I3 = i * 3;
            Matrix3x3 H  = H3Nx3N.template block<3, 3>(I3, I3);
            int       L  = indices(i);

            if(is_fixed(L))
                continue;

            muda::eigen::atomic_add(diag(i), H);
        }
    }

    template <int N>
    __device__ void make_spd(Matrix<Float, N, N>& H)
    {
        Vector<Float, N>    eigen_values;
        Matrix<Float, N, N> eigen_vectors;
        muda::eigen::template evd<Float, N>(H, eigen_values, eigen_vectors);
        for(int i = 0; i < N; ++i)
        {
            auto& v = eigen_values(i);
            v       = v < 0.0 ? 0.0 : v;
        }
        H = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    }

}  // namespace detail


void FEMLinearSubsystem::Impl::_assemble_gradient(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;
    // Kinetic
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(fem().G3s.size(),
               [G3s = fem().G3s.cviewer().name("G3s"),
                gradient = info.gradient().viewer().name("gradient")] __device__(int i) mutable
               {
                   // fill gradient
                   gradient.segment<3>(i * 3) = G3s(i);
               });

    // Elastic

    // Codim1D
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(fem().G6s.size(),
               [G6s      = fem().G6s.cviewer().name("G6s"),
                indices  = fem().codim_1ds.cviewer().name("codim_1d_indices"),
                gradient = info.gradient().viewer().name("gradient"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
               {
                   // fill gradient
                   detail::fill_gradient<2>(gradient, G6s(I), indices(I), is_fixed);
               });

    // Codim2D
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(fem().G9s.size(),
               [G9s      = fem().G9s.cviewer().name("G9s"),
                indices  = fem().codim_2ds.cviewer().name("codim_2d_indices"),
                gradient = info.gradient().viewer().name("gradient"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
               {
                   // fill gradient
                   detail::fill_gradient<3>(gradient, G9s(I), indices(I), is_fixed);
               });

    // FEM3D
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(fem().G12s.size(),
               [G12s     = fem().G12s.cviewer().name("G12s"),
                indices  = fem().tets.cviewer().name("tets"),
                gradient = info.gradient().viewer().name("gradient"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
               {
                   // fill gradient
                   detail::fill_gradient<4>(gradient, G12s(I), indices(I), is_fixed);
               });

    // Extra
    auto EG = fem().extra_constitution_gradient.view();
    ParallelFor()
        .kernel_name(__FUNCTION__)
        .apply(fem().extra_constitution_gradient.doublet_count(),
               [extra_constitution_gradient = EG.cviewer().name("extra_gradient"),
                gradient = info.gradient().viewer().name("gradient"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
               {
                   const auto& [i, G3] = extra_constitution_gradient(I);

                   if(is_fixed(i))
                   {
                       //
                   }
                   else
                   {
                       gradient.segment<3>(i * 3).atomic_add(G3);
                   }
               });

    if(fem_contact_receiver)  //  if contact enabled
    {
        auto contact_count = contact().contact_gradient.doublet_count();
        if(contact_count)
        {
            ParallelFor()
                .kernel_name(__FUNCTION__)
                .apply(contact_count,
                       [contact_gradient = contact().contact_gradient.cviewer().name("contact_gradient"),
                        gradient = info.gradient().viewer().name("gradient"),
                        vertex_offset = finite_element_vertex_reporter->vertex_offset(),
                        is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
                       {
                           const auto& [g_i, G3] = contact_gradient(I);
                           auto i = g_i - vertex_offset;  // from global to local

                           if(is_fixed(i))
                           {
                               //
                           }
                           else
                           {
                               gradient.segment<3>(i * 3).atomic_add(G3);
                           }
                       });
        }
    }
}

void FEMLinearSubsystem::Impl::_assemble_hessian(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;

    IndexT offset    = 0;
    auto   dst_H3x3s = info.hessian();

    // Kinetic
    {
        auto N = fem().H3x3s.size();
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(fem().H3x3s.size(),
                   [H3x3s = fem().H3x3s.cviewer().name("H3x3s"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    diag_hessians = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       // Diag
                       hessian(I).write(I, I, H3x3s(I));
                       diag_hessians(I) = H3x3s(I);

                       // cout << "Kinetic hessian: \n" << H3x3s(I) << "\n";
                   });

        offset += N;
    }


    // Elastic

    {  // Codim1D
        auto N = fem().H6x6s.size() * H6x6_to_H3x3;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(fem().H6x6s.size(),
                   [H6x6s = fem().H6x6s.cviewer().name("H6x6s"),
                    indices = fem().codim_1ds.cviewer().name("codim_1d_indices"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    is_fixed      = fem().is_fixed.cviewer().name("is_fixed"),
                    diag_hessians = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       // fill hessian
                       Matrix6x6 H = H6x6s(I);
                       detail::make_spd<6>(H);
                       detail::fill_hessian<2>(I, hessian, H, indices(I), is_fixed);
                       detail::fill_diag_hessian<2>(diag_hessians, H, indices(I), is_fixed);
                   });

        offset += N;
    }

    {  // Codim2D

        auto N = fem().H9x9s.size() * H9x9_to_H3x3;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(fem().H9x9s.size(),
                   [H9x9s = fem().H9x9s.cviewer().name("H9x9s"),
                    indices = fem().codim_2ds.cviewer().name("codim_2d_indices"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    is_fixed      = fem().is_fixed.cviewer().name("is_fixed"),
                    diag_hessians = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       // fill hessian
                       Matrix9x9 H = H9x9s(I);
                       detail::make_spd<9>(H);
                       detail::fill_hessian<3>(I, hessian, H, indices(I), is_fixed);
                       detail::fill_diag_hessian<3>(diag_hessians, H, indices(I), is_fixed);
                   });

        offset += N;
    }


    {  // FEM3D
        auto N = fem().H12x12s.size() * H12x12_to_H3x3;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(fem().H12x12s.size(),
                   [H12x12s = fem().H12x12s.cviewer().name("H12x12s"),
                    indices = fem().tets.cviewer().name("tets"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    is_fixed      = fem().is_fixed.cviewer().name("is_fixed"),
                    diag_hessians = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       // fill hessian
                       Matrix12x12 H = H12x12s(I);
                       detail::make_spd<12>(H);
                       detail::fill_hessian<4>(I, hessian, H, indices(I), is_fixed);
                       detail::fill_diag_hessian<4>(diag_hessians, H, indices(I), is_fixed);
                   });

        offset += N;
    }

    {  // Extra
        auto EH = fem().extra_constitution_hessian.view();
        auto N  = fem().extra_constitution_hessian.triplet_count();

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(fem().extra_constitution_hessian.triplet_count(),
                   [extra_constitution_hessian = EH.cviewer().name("extra_hessian"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    is_fixed      = fem().is_fixed.cviewer().name("is_fixed"),
                    diag_hessians = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       const auto& [i, j, H3] = extra_constitution_hessian(I);

                       if(is_fixed(i) || is_fixed(j))
                       {
                           hessian(I).write(i, j, Matrix3x3::Zero());
                       }
                       else
                       {
                           hessian(I).write(i, j, H3);

                           if(i == j)
                               muda::eigen::atomic_add(diag_hessians(i), H3);
                       }
                   });

        offset += N;
    }


    if(fem_contact_receiver)  // if contact enabled
    {
        auto contact_count = contact().contact_hessian.triplet_count();
        auto N             = contact_count;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(contact_count,
                   [contact_hessian = contact().contact_hessian.cviewer().name("contact_hessian"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    vertex_offset = finite_element_vertex_reporter->vertex_offset(),
                    is_fixed      = fem().is_fixed.cviewer().name("is_fixed"),
                    diag_hessians = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       const auto& [g_i, g_j, H3] = contact_hessian(I);
                       auto i                     = g_i - vertex_offset;
                       auto j                     = g_j - vertex_offset;

                       if(is_fixed(i) || is_fixed(j))
                       {
                           hessian(I).write(i, j, Matrix3x3::Zero());
                       }
                       else
                       {
                           hessian(I).write(i, j, H3);

                           if(i == j)
                               muda::eigen::atomic_add(diag_hessians(i), H3);
                       }
                   });

        offset += N;
    }

    // UIPC_ASSERT(offset == animator_hessian_offset, "Hessian size mismatch");
}

void FEMLinearSubsystem::Impl::_assemble_animation(GlobalLinearSystem::DiagInfo& info)
{
    using namespace muda;
    if(finite_element_animator)
    {
        FiniteElementAnimator::AssembleInfo this_info;
        this_info.hessians =
            info.hessian().subview(animator_hessian_offset, animator_hessian_count);
        finite_element_animator->assemble(this_info);

        // setup gradient
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(this_info.gradients.doublet_count(),
                   [anim_gradients = this_info.gradients.cviewer().name("gradients"),
                    gradient = info.gradient().viewer().name("gradient"),
                    is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
                   {
                       const auto& [i, G3] = anim_gradients(I);
                       if(is_fixed(i))
                       {
                           //
                       }
                       else
                       {
                           gradient.segment<3>(i * 3).atomic_add(G3);
                       }
                   });

        // setup hessian
        auto dst_hessian =
            info.hessian().subview(animator_hessian_offset, animator_hessian_count);
        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(this_info.hessians.triplet_count(),
                   [anim_hessians = this_info.hessians.cviewer().name("hessians"),
                    hessian      = dst_hessian.viewer().name("hessian"),
                    is_fixed     = fem().is_fixed.cviewer().name("is_fixed"),
                    diag_hessian = fem().diag_hessians.viewer().name(
                        "diag_hessian")] __device__(int I) mutable
                   {
                       const auto& [i, j, H3] = anim_hessians(I);
                       if(is_fixed(i) || is_fixed(j))
                       {
                           hessian(I).write(i, j, Matrix3x3::Zero());
                       }
                       else
                       {
                           hessian(I).write(i, j, H3);

                           if(i == j)
                               muda::eigen::atomic_add(diag_hessian(i), H3);
                       }
                   });
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
        .kernel_name(__FUNCTION__)
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

}  // namespace uipc::backend::cuda
