#include <finite_element/fem_linear_subsystem.h>
#include <sim_engine.h>
#include <kernel_cout.h>
#include <muda/ext/eigen.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
REGISTER_SIM_SYSTEM(FEMLinearSubsystem);

void FEMLinearSubsystem::do_build(DiagLinearSubsystem::BuildInfo&)
{
    m_impl.finite_element_method = &require<FiniteElementMethod>();
    m_impl.finite_element_vertex_reporter = &require<FiniteElementVertexReporter>();

    m_impl.fem_contact_receiver = find<FEMContactReceiver>();

    m_impl.converter.reserve_ratio(1.1);
}

void FEMLinearSubsystem::Impl::report_extent(GlobalLinearSystem::DiagExtentInfo& info)
{
    UIPC_ASSERT(info.storage_type() == GlobalLinearSystem::HessianStorageType::Full,
                "Now only support Full Hessian");

    // 1) Hessian Count
    auto hessian_block_count =
        fem().H12x12s.size() * H12x12_to_H3x3 + fem().H9x9s.size() * H9x9_to_H3x3
        + fem().H6x6s.size() * H6x6_to_H3x3 + fem().H3x3s.size();

    if(fem_contact_receiver)  // if contact enabled
    {
        auto contact_count = contact().contact_hessian.triplet_count();
        hessian_block_count += contact_count;
    }

    // 2) Gradient Count
    auto dof_count = fem().dxs.size() * 3;

    info.extent(hessian_block_count, dof_count);
}

void FEMLinearSubsystem::Impl::assemble(GlobalLinearSystem::DiagInfo& info)
{
    _assemble_gradient(info);
    _assemble_hessian(info);
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
               [G3s      = fem().G3s.cviewer().name("G3s"),
                gradient = info.gradient().viewer().name("gradient"),
                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int i) mutable
               {
                   if(is_fixed(i))
                   {
                       gradient.segment<3>(i * 3) = Vector3::Zero().eval();
                   }
                   else
                   {
                       gradient.segment<3>(i * 3) = G3s(i);
                   }

                   // gradient.segment<3>(i * 3) = Vector3::Zero().eval();
               });

    // Elastic

    //// Codim1D
    //ParallelFor()
    //    .kernel_name(__FUNCTION__)
    //    .apply(fem().G6s.size(),
    //           [G6s      = fem().G6s.cviewer().name("G6s"),
    //            indices  = fem().codim_1ds.cviewer().name("codim_1d_indices"),
    //            gradient = info.gradient().viewer().name("gradient"),
    //            is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
    //           {
    //               // fill gradient
    //               detail::fill_gradient<2>(gradient, G6s(I), indices(I), is_fixed);
    //           });

    //// Codim2D
    //ParallelFor()
    //    .kernel_name(__FUNCTION__)
    //    .apply(fem().G9s.size(),
    //           [G9s      = fem().G9s.cviewer().name("G9s"),
    //            indices  = fem().codim_2ds.cviewer().name("codim_2d_indices"),
    //            gradient = info.gradient().viewer().name("gradient"),
    //            is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
    //           {
    //               // fill gradient
    //               detail::fill_gradient<3>(gradient, G9s(I), indices(I), is_fixed);
    //           });

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
                   [H3x3s   = fem().H3x3s.cviewer().name("H3x3s"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name(
                        "hessian")] __device__(int I) mutable
                   {
                       // Diag
                       hessian(I).write(I, I, H3x3s(I));
                       // cout << "Kinetic hessian: \n" << H3x3s(I) << "\n";
                   });

        offset += N;
    }


    // Elastic

    //{  // Codim1D
    //    auto N = fem().H6x6s.size() * H6x6_to_H3x3;

    //    ParallelFor()
    //        .kernel_name(__FUNCTION__)
    //        .apply(fem().H6x6s.size(),
    //               [H6x6s = fem().H6x6s.cviewer().name("H6x6s"),
    //                indices = fem().codim_1ds.cviewer().name("codim_1d_indices"),
    //                hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
    //                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
    //               {
    //                   // fill hessian
    //                   Matrix6x6 H = H6x6s(I);
    //                   detail::make_spd<6>(H);
    //                   detail::fill_hessian<2>(I, hessian, H, indices(I), is_fixed);
    //               });

    //    offset += N;
    //}

    //{  // Codim2D

    //    auto N = fem().H9x9s.size() * H9x9_to_H3x3;

    //    ParallelFor()
    //        .kernel_name(__FUNCTION__)
    //        .apply(fem().H9x9s.size(),
    //               [H9x9s = fem().H9x9s.cviewer().name("H9x9s"),
    //                indices = fem().codim_2ds.cviewer().name("codim_2d_indices"),
    //                hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
    //                is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
    //               {
    //                   // fill hessian
    //                   Matrix9x9 H = H9x9s(I);
    //                   detail::make_spd<9>(H);
    //                   detail::fill_hessian<3>(I, hessian, H, indices(I), is_fixed);
    //               });

    //    offset += N;
    //}


    {  // FEM3D
        auto N = fem().H12x12s.size() * H12x12_to_H3x3;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(fem().H12x12s.size(),
                   [H12x12s = fem().H12x12s.cviewer().name("H12x12s"),
                    indices = fem().tets.cviewer().name("tets"),
                    hessian = dst_H3x3s.subview(offset, N).viewer().name("hessian"),
                    is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
                   {
                       // fill hessian
                       Matrix12x12 H = H12x12s(I);
                       detail::make_spd<12>(H);
                       detail::fill_hessian<4>(I, hessian, H, indices(I), is_fixed);
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
                    hessian = info.hessian().viewer().name("hessian"),
                    vertex_offset = finite_element_vertex_reporter->vertex_offset(),
                    is_fixed = fem().is_fixed.cviewer().name("is_fixed")] __device__(int I) mutable
                   {
                       const auto& [g_i, g_j, H3] = contact_hessian(I);
                       auto i                     = g_i - vertex_offset;
                       auto j                     = g_j - vertex_offset;

                       if(is_fixed(i) || is_fixed(j))
                       {
                           //
                       }
                       else
                       {
                           hessian(I).write(i, j, H3);
                       }
                   });

        offset += N;
    }

    UIPC_ASSERT(offset == info.hessian().triplet_count(), "Hessian size mismatch");
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
                   cout << "solution dx(" << i << "):" << dxs(i).transpose().eval() << "\n";
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
