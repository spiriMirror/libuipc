#include <contact_system/simplex_contact_constitution.h>
#include <collision_detection/global_dcd_filter.h>
#include <collision_detection/simplex_dcd_filter.h>
#include <muda/ext/eigen/evd.h>

namespace uipc::backend::cuda
{
void SimplexContactConstitution::do_build()
{
    m_impl.global_dcd_filter      = &require<GlobalDCDFilter>();
    m_impl.global_contact_manager = &require<GlobalContactManager>();
    m_impl.global_vertex_manager  = &require<GlobalVertexManager>();

    BuildInfo info;
    do_build(info);

    m_impl.global_contact_manager->add_reporter(this);
    m_impl.dt = world().scene().info()["dt"].get<Float>();
}

void SimplexContactConstitution::do_compute_energy(GlobalContactManager::EnergyInfo& info)
{
    EnergyInfo this_info{&m_impl};

    auto dcd_filter = m_impl.global_dcd_filter->simplex_filter();

    m_impl.PT_count = dcd_filter->PTs().size();
    m_impl.EE_count = dcd_filter->EEs().size();
    m_impl.PE_count = dcd_filter->PEs().size();
    m_impl.PP_count = dcd_filter->PPs().size();

    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;
    m_impl.energies.resize(count_4 + count_3 + count_2);

    SizeT offset            = 0;
    this_info.m_PT_energies = m_impl.energies.view(offset, m_impl.PT_count);
    offset += m_impl.PT_count;
    this_info.m_EE_energies = m_impl.energies.view(offset, m_impl.EE_count);
    offset += m_impl.EE_count;
    this_info.m_PE_energies = m_impl.energies.view(offset, m_impl.PE_count);
    offset += m_impl.PE_count;
    this_info.m_PP_energies = m_impl.energies.view(offset, m_impl.PP_count);


    do_compute_energy(this_info);

    using namespace muda;
    DeviceReduce().Sum(
        m_impl.energies.data(), info.energy().data(), m_impl.energies.size());
}

void SimplexContactConstitution::do_report_extent(GlobalContactManager::ContactExtentInfo& info)
{
    auto dcd_filter = m_impl.global_dcd_filter->simplex_filter();

    m_impl.PT_count = dcd_filter->PTs().size();
    m_impl.EE_count = dcd_filter->EEs().size();
    m_impl.PE_count = dcd_filter->PEs().size();
    m_impl.PP_count = dcd_filter->PPs().size();

    auto count_4 = (m_impl.PT_count + m_impl.EE_count);
    auto count_3 = m_impl.PE_count;
    auto count_2 = m_impl.PP_count;

    // expand to hessian3x3 and graident3
    SizeT contact_gradient_count = 4 * count_4 + 3 * count_3 + 2 * count_2;
    SizeT contact_hessian_count = 4 * 4 * count_4 + 3 * 3 * count_3 + 2 * 2 * count_2;

    info.gradient_count(contact_gradient_count);
    info.hessian_count(contact_hessian_count);


    m_impl.loose_resize(m_impl.PT_EE_indices, count_4);
    m_impl.loose_resize(m_impl.PT_EE_gradients, count_4);
    m_impl.loose_resize(m_impl.PT_EE_hessians, count_4);

    m_impl.loose_resize(m_impl.PE_indices, count_3);
    m_impl.loose_resize(m_impl.PE_gradients, count_3);
    m_impl.loose_resize(m_impl.PE_hessians, count_3);

    m_impl.loose_resize(m_impl.PP_indices, count_2);
    m_impl.loose_resize(m_impl.PP_gradients, count_2);
    m_impl.loose_resize(m_impl.PP_hessians, count_2);
}


void SimplexContactConstitution::do_assemble(GlobalContactManager::ContactInfo& info)
{
    ContactInfo this_info{&m_impl};

    this_info.m_PT_gradients = m_impl.PT_EE_gradients.view(0, m_impl.PT_count);
    this_info.m_EE_gradients =
        m_impl.PT_EE_gradients.view(m_impl.PT_count, m_impl.EE_count);
    this_info.m_PE_gradients = m_impl.PE_gradients.view();
    this_info.m_PP_gradients = m_impl.PP_gradients.view();

    this_info.m_PT_hessians = m_impl.PT_EE_hessians.view(0, m_impl.PT_count);
    this_info.m_EE_hessians = m_impl.PT_EE_hessians.view(m_impl.PT_count, m_impl.EE_count);
    this_info.m_PE_hessians = m_impl.PE_hessians.view();
    this_info.m_PP_hessians = m_impl.PP_hessians.view();

    // let subclass to fill in the data
    do_assemble(this_info);

    // _assemble the data to the global contact manager
    m_impl.assemble(info);
}

muda::CBuffer2DView<ContactCoeff> SimplexContactConstitution::BaseInfo::contact_tabular() const
{
    return m_impl->global_contact_manager->contact_tabular();
}

muda::CBufferView<Vector4i> SimplexContactConstitution::BaseInfo::PTs() const
{
    return m_impl->simplex_dcd_filter()->PTs();
}

muda::CBufferView<Vector4i> SimplexContactConstitution::BaseInfo::EEs() const
{
    return m_impl->simplex_dcd_filter()->EEs();
}

muda::CBufferView<Vector3i> SimplexContactConstitution::BaseInfo::PEs() const
{
    return m_impl->simplex_dcd_filter()->PEs();
}

muda::CBufferView<Vector2i> SimplexContactConstitution::BaseInfo::PPs() const
{
    return m_impl->simplex_dcd_filter()->PPs();
}

muda::CBufferView<Vector3> SimplexContactConstitution::BaseInfo::positions() const
{
    return m_impl->global_vertex_manager->positions();
}

muda::CBufferView<Vector3> SimplexContactConstitution::BaseInfo::rest_positions() const
{
    return m_impl->global_vertex_manager->rest_positions();
}

Float SimplexContactConstitution::BaseInfo::d_hat() const
{
    return m_impl->global_contact_manager->d_hat();
}

Float SimplexContactConstitution::BaseInfo::dt() const
{
    return m_impl->dt;
}

namespace detail
{
    template <SizeT N>
    __inline__ __device__ void fill_contact_hessian(muda::TripletMatrixViewer<Float, 3>& H3x3,
                                                    int                      I,
                                                    const Vector<IndexT, N>& D,
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

    template <SizeT N>
    __inline__ __device__ void fill_contact_gradient(muda::DoubletVectorViewer<Float, 3>& G3,
                                                     int                      I,
                                                     const Vector<IndexT, N>& D,
                                                     const Vector<Float, 3 * N>& G)
    {
        auto offset = N * I;
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto L = D(i);
            G3(offset++).write(L, G.segment<3>(3 * i));
        }
    }

    template <SizeT N>
    __inline__ __device__ void make_spd(Matrix<Float, N, N>& mat)
    {
        Vector<Float, N>    eigen_values;
        Matrix<Float, N, N> eigen_vectors;
        muda::eigen::template evd(mat, eigen_values, eigen_vectors);
#pragma unroll
        for(int i = 0; i < N; ++i)
        {
            auto& v = eigen_values(i);
            v       = v < 0.0 ? 0.0 : v;
        }
        mat = eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.transpose();
    }
}  // namespace detail


void SimplexContactConstitution::Impl::assemble(GlobalContactManager::ContactInfo& info)
{
    using namespace muda;

    auto H3x3 = info.hessian();
    auto G3   = info.gradient();
    auto PTs  = simplex_dcd_filter()->PTs();
    auto EEs  = simplex_dcd_filter()->EEs();
    auto PEs  = simplex_dcd_filter()->PEs();
    auto PPs  = simplex_dcd_filter()->PPs();

    auto PT_hessian  = PT_EE_hessians.view(0, PTs.size());
    auto PT_gradient = PT_EE_gradients.view(0, PTs.size());
    auto EE_hessian  = PT_EE_hessians.view(PTs.size(), EEs.size());
    auto EE_gradient = PT_EE_gradients.view(PTs.size(), EEs.size());

    SizeT H3x3_offset = 0;
    SizeT G3_offset   = 0;

    // PT
    {
        SizeT H3x3_count = PTs.size() * 16;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-PT_H12x12")
            .apply(PT_hessian.size(),
                   [PT_H12x12s = PT_hessian.cviewer().name("H12x12"),
                    PTs        = PTs.cviewer().name("PTs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix12x12 H12x12 = PT_H12x12s(I);
                       const auto& D4     = PTs(I);
                       detail::make_spd(H12x12);
                       detail::fill_contact_hessian<4>(H3x3, I, D4, H12x12);
                   });
        H3x3_offset += H3x3_count;

        SizeT G3_count = PT_gradient.size() * 4;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-PT_G12")
            .apply(PT_gradient.size(),
                   [PT_G12s = PT_gradient.cviewer().name("G12"),
                    PTs     = PTs.cviewer().name("PTs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G12 = PT_G12s(I);
                       const auto& D4  = PTs(I);
                       detail::fill_contact_gradient<4>(G3, I, D4, G12);
                   });
        G3_offset += G3_count;
    }

    // EE
    {
        SizeT H3x3_count = EEs.size() * 16;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-EE_H12x12")
            .apply(EE_hessian.size(),
                   [EE_H12x12s = EE_hessian.cviewer().name("H12x12"),
                    EEs        = EEs.cviewer().name("EEs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix12x12 H12x12 = EE_H12x12s(I);
                       const auto& D4     = EEs(I);
                       detail::make_spd(H12x12);
                       detail::fill_contact_hessian<4>(H3x3, I, D4, H12x12);
                   });

        H3x3_offset += H3x3_count;

        SizeT G3_count = EE_gradient.size() * 4;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-EE_G12")
            .apply(EE_gradient.size(),
                   [G12s = EE_gradient.cviewer().name("G12"),
                    EEs  = EEs.cviewer().name("EEs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G12 = G12s(I);
                       const auto& D4  = EEs(I);
                       detail::fill_contact_gradient<4>(G3, I, D4, G12);
                   });

        G3_offset += G3_count;
    }

    // PE
    {
        SizeT H3x3_count = PEs.size() * 9;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-PE_H9x9")
            .apply(PEs.size(),
                   [PE_H9x9s = PE_hessians.cviewer().name("H9x9"),
                    PEs      = PEs.cviewer().name("PEs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix9x9   H9x9 = PE_H9x9s(I);
                       const auto& D3   = PEs(I);
                       detail::make_spd(H9x9);
                       detail::fill_contact_hessian<3>(H3x3, I, D3, H9x9);
                   });

        H3x3_offset += H3x3_count;

        SizeT G3_count = PEs.size() * 3;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-PE_G9")
            .apply(PEs.size(),
                   [G9s = PE_gradients.cviewer().name("G9"),
                    PEs = PEs.cviewer().name("PEs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G9 = G9s(I);
                       const auto& D3 = PEs(I);
                       detail::fill_contact_gradient<3>(G3, I, D3, G9);
                   });

        G3_offset += G3_count;
    }


    // PP
    {
        SizeT H3x3_count = PPs.size() * 4;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-PP_H6x6")
            .apply(PPs.size(),
                   [PP_H6x6s = PP_hessians.cviewer().name("H6x6"),
                    PPs      = PPs.cviewer().name("PPs"),
                    H3x3 = H3x3.subview(H3x3_offset, H3x3_count).viewer().name("H3x3")] __device__(int I) mutable
                   {
                       Matrix6x6   H6x6 = PP_H6x6s(I);
                       const auto& D2   = PPs(I);
                       detail::make_spd(H6x6);
                       detail::fill_contact_hessian<2>(H3x3, I, D2, H6x6);
                   });

        H3x3_offset += H3x3_count;

        SizeT G3_count = PPs.size() * 2;
        ParallelFor()
            .kernel_name(__FUNCTION__ "-PP_G6")
            .apply(PPs.size(),
                   [G6s = PP_gradients.cviewer().name("G6"),
                    PPs = PPs.cviewer().name("PPs"),
                    G3 = G3.subview(G3_offset, G3_count).viewer().name("G3")] __device__(int I) mutable
                   {
                       const auto& G6 = G6s(I);
                       const auto& D2 = PPs(I);
                       detail::fill_contact_gradient<2>(G3, I, D2, G6);
                   });

        G3_offset += G3_count;
    }

    UIPC_ASSERT(H3x3_offset == info.hessian().triplet_count(), "size mismatch");
    UIPC_ASSERT(G3_offset == info.gradient().doublet_count(), "size mismatch");
}

SimplexDCDFilter* SimplexContactConstitution::Impl::simplex_dcd_filter() const noexcept
{
    return global_dcd_filter->simplex_filter();
}
}  // namespace uipc::backend::cuda
