#include <contact_system/simplex_contact_constitution.h>

namespace uipc::backend::cuda
{
void SimplexContactConstitution::do_build() {}

void SimplexContactConstitution::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    PrimitiveCountInfo info{&m_impl};
    do_report_extent(info);
    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;

    // expand to hessian3x3 and graident3
    SizeT contact_gradient_count = 4 * count_4 + 3 * count_3 + 2 * count_2;
    SizeT contact_hessian_count = 4 * 4 * count_4 + 3 * 3 * count_3 + 2 * 2 * count_2;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(contact_hessian_count);
}

void SimplexContactConstitution::do_assemble(GlobalContactManager::ContactInfo& info)
{
    m_impl.prepare();
    ContactInfo this_info{&m_impl};

    // let subclass to fill in the data
    do_assemble(this_info);

    // assemble the data to the global contact manager
    m_impl.assemble(info);
}

void SimplexContactConstitution::Impl::prepare()
{
    auto count_4 = (PT_count + EE_count);
    auto count_3 = PE_count;
    auto count_2 = PP_count;

    PT_EE_hessians.resize(count_4);
    PT_EE_gradients.resize(count_4);

    PE_hessians.resize(count_3);
    PE_gradients.resize(count_3);

    PP_hessians.resize(count_2);
    PP_gradients.resize(count_2);
}


namespace detail
{
    template <SizeT N>
    __inline__ __device__ void fill_contact_hessian(muda::TripletMatrixViewer<Float, 3>& H3x3,
                                                    int                     I,
                                                    const Vector<Float, N>& D,
                                                    const Matrix<Float, 3 * N, 3 * N>& H)
    {
        auto offset = N * N * I;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto L = D(i);
#pragma unroll
            for(int j = 0; j < N; ++j)
            {
                auto R = D(j);
                H3x3(offset++).write(L, R, H.block<3, 3>(3 * i, 3 * j));
            }
        }
    }
}  // namespace detail


void SimplexContactConstitution::Impl::assemble(GlobalContactManager::ContactInfo& info)
{
    using namespace muda;

    auto H3x3 = info.hessian();
    auto G3   = info.gradient();

    auto H12x12_count = PT_EE_hessians.size();
    auto G12_count    = PT_EE_gradients.size();
    auto H9x9_count   = PE_hessians.size();
    auto G9_count     = PE_gradients.size();
    auto H6x6_count   = PP_hessians.size();
    auto G6_count     = PP_gradients.size();

    SizeT H3x3_offset = 0;
    SizeT G3_offset   = 0;

    {
        SizeT H3x3_count = H12x12_count * 16;
        SizeT G3_count   = G12_count * 4;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(H12x12_count,
                   [H12x12s = PT_EE_hessians.cviewer().name("H12x12"),
                    D4s     = PT_EE_indices.cviewer().name("D4"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       auto H12x12 = H12x12s(I);
                       auto D4     = D4s(I);
                       detail::fill_contact_hessian<4>(H3x3, I, D4, H12x12);
                   });


        H3x3_offset += H3x3_count;
        G3_offset += G3_count;
    }

    {
        SizeT H3x3_count = H9x9_count * 9;
        SizeT G3_count   = G9_count * 3;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(H9x9_count,
                   [H9x9s = PE_hessians.cviewer().name("H9x9"),
                    D3s   = PE_indices.cviewer().name("D3"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       auto H9x9 = H9x9s(I);
                       auto D3   = D3s(I);
                       detail::fill_contact_hessian<3>(H3x3, I, D3, H9x9);
                   });

        H3x3_offset += H3x3_count;
        G3_offset += G3_count;
    }

    {
        SizeT H3x3_count = H6x6_count * 4;
        SizeT G3_count   = G6_count * 2;

        ParallelFor()
            .kernel_name(__FUNCTION__)
            .apply(H6x6_count,
                   [H6x6s = PP_hessians.cviewer().name("H6x6"),
                    D2s   = PP_indices.cviewer().name("D2"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       auto H6x6 = H6x6s(I);
                       auto D2   = D2s(I);
                       detail::fill_contact_hessian<2>(H3x3, I, D2, H6x6);
                   });

        H3x3_offset += H3x3_count;
        G3_offset += G3_count;
    }

    UIPC_ASSERT(H3x3_offset == info.hessian().triplet_count(), "size mismatch");
    UIPC_ASSERT(G3_offset == info.gradient().doublet_count(), "size mismatch");
}  // namespace uipc::backend::cuda
